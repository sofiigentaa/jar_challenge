"""Experimental autonomous rescue: AMCL, grid coverage, RGB/LiDAR and safe return."""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener, TransformException

from equipo_jar.estrategia import Grid, Victims, speed_limit, wrap
from equipo_jar.reportes import armar_reporte

QOS_MAPA = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def matrix(t):
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return rotation, np.array([t.translation.x, t.translation.y, t.translation.z])


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class NodoEquipo(Node):
    def __init__(self):
        super().__init__('nodo_equipo')
        defaults = dict(frecuencia_control_hz=15.0, velocidad_lineal_max=.12,
                        velocidad_angular_max=.55, cantidad_victimas=5,
                        radio_robot=.28, margen_seguridad=.06, desaceleracion=.25,
                        timeout_sensor=.4, paso_cobertura=.65, tolerancia_meta=.12,
                        regreso_segundos=450.0, correccion_centro_caja=.10,
                        distancia_reporte_max=1.5, sigma_localizacion_max=.10,
                        base_frame='base_link', separacion_reportes=.75)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = lambda name: self.get_parameter(name).value
        self.grid = None
        self.start_time = None
        self.home = None
        self.scan = None
        self.scan_wall = -math.inf
        self.info = None
        self.image_wall = -math.inf
        self.amcl = None
        self.amcl_wall = -math.inf
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.victims = Victims(self.p('cantidad_victimas'), self.p('separacion_reportes'))
        self.goal = None
        self.route = []
        self.targets = []
        self.covered = set()
        self.returning = False
        self.finished = False
        self.last_plan = -math.inf
        self.sweep = 0.0
        self.sweep_last = None
        self.sweeping = False
        self.home_ticks = 0
        self.command = (0., 0.)
        self.last_notice = -math.inf
        self.observe_until = -math.inf
        self.next_observation = -math.inf
        self.create_subscription(OccupancyGrid, '/map', self.on_map, QOS_MAPA)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/cam_1/color/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/cam_1/color/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_victimas = self.create_publisher(PointStamped, '/victimas', 10)
        self.pub_done = self.create_publisher(Empty, '/done', 10)
        self.create_timer(1/self.p('frecuencia_control_hz'), self.control)
        self.get_logger().info('Esperando /map. Versión experimental: validar con juez en Gazebo.')

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def notice(self, text):
        if time.monotonic()-self.last_notice > 3:
            self.get_logger().warn(text)
            self.last_notice = time.monotonic()

    def send(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.pub_cmd_vel.publish(msg)
        self.command = (v, w)

    def on_map(self, msg):
        if self.grid is not None:
            return  # one mission per process, never reset report budget on republish
        if msg.header.frame_id != 'map' or msg.info.resolution <= 0:
            self.get_logger().error('Mapa inválido: se necesita marco map y resolución positiva.')
            return
        data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        p = msg.info.origin.position
        self.grid = Grid(data, msg.info.resolution,
                         (p.x, p.y, yaw(msg.info.origin.orientation)), self.p('radio_robot')+self.p('margen_seguridad'))
        step = max(1, round(self.p('paso_cobertura')/msg.info.resolution))
        self.targets = [(r,c) for r in range(0,msg.info.height,step)
                        for c in range(0,msg.info.width,step) if self.grid.free[r,c]]
        self.start_time = self.now_s()
        self.get_logger().info(f'Mapa recibido; {len(self.targets)} puntos de observación derivados del mapa.')

    def on_scan(self, msg):
        self.scan, self.scan_wall = msg, time.monotonic()

    def on_info(self, msg):
        self.info = msg

    def on_amcl(self, msg):
        self.amcl, self.amcl_wall = msg, time.monotonic()

    def localized(self):
        if self.amcl is None or time.monotonic()-self.amcl_wall > 2.0:
            return False
        cov = self.amcl.pose.covariance
        return (0 <= cov[0] <= self.p('sigma_localizacion_max')**2 and
                0 <= cov[7] <= self.p('sigma_localizacion_max')**2 and
                0 <= cov[35] < .12)

    def transform(self, target, source, stamp):
        t = self.tf.lookup_transform(target, source, Time.from_msg(stamp))
        return matrix(t.transform)

    def scan_points(self, frame):
        s = self.scan
        ranges = np.asarray(s.ranges)
        angles = s.angle_min + np.arange(len(ranges))*s.angle_increment
        valid = np.isfinite(ranges) & (ranges >= s.range_min) & (ranges <= s.range_max)
        points = np.column_stack((ranges[valid]*np.cos(angles[valid]),
                                  ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum())))
        rot, tr = self.transform(frame, s.header.frame_id, s.header.stamp)
        return points@rot.T + tr

    def on_image(self, msg):
        self.image_wall = time.monotonic()
        if self.grid is None or not self.localized() or self.scan is None or self.info is None:
            return
        if abs(seconds(msg.header.stamp)-seconds(self.scan.header.stamp)) > .12:
            return
        if time.monotonic()-self.scan_wall > self.p('timeout_sensor'):
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if (image.shape[1],image.shape[0]) != (self.info.width,self.info.height):
                return
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0,110,70), (10,255,255)) | cv2.inRange(hsv, (170,110,70), (179,255,255))
            mask = cv2.erode(mask, np.ones((3,3),np.uint8))
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
            points = self.scan_points(self.info.header.frame_id)
            front = points[:,2] > .05
            points = points[front]
            if not len(points):
                return
            if self.info.distortion_model not in ('plumb_bob', ''):
                return
            pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3),
                                          np.array(self.info.k).reshape(3,3), np.array(self.info.d))
            pixels = np.rint(pixels[:,0,:]).astype(int)
            valid = ((pixels[:,0]>=0)&(pixels[:,0]<mask.shape[1])&
                     (pixels[:,1]>=0)&(pixels[:,1]<mask.shape[0]))
            points, pixels = points[valid], pixels[valid]
            rot, tr = self.transform('map', self.info.header.frame_id, self.scan.header.stamp)
            for label in range(1,count):
                if stats[label,cv2.CC_STAT_AREA] < 30:
                    continue
                chosen = points[labels[pixels[:,1],pixels[:,0]] == label]
                if len(chosen) < 3:
                    continue
                depth = chosen[:,2]
                if np.ptp(depth) > .15 or np.median(depth) > self.p('distancia_reporte_max'):
                    continue
                surface = np.median(chosen,axis=0)
                # Generic box half-size correction, not robot approach distance.
                # Must be calibrated against actual victim dimensions in practice.
                horizontal = np.array([surface[0],0.,surface[2]])
                surface += self.p('correccion_centro_caja')*horizontal/np.linalg.norm(horizontal)
                position = rot@surface + tr
                existing = [t for t in self.victims.tracks
                            if np.linalg.norm(t.center-position[:2]) < self.victims.association]
                if not any(t.reported for t in existing) and self.now_s() >= self.next_observation:
                    self.observe_until = self.now_s()+2.0
                    self.next_observation = self.now_s()+6.0
                # Stop and collect distinct, synchronized frames before publishing.
                if abs(self.command[0]) > .015 or abs(self.command[1]) > .10:
                    continue
                report = self.victims.observe(position[:2], seconds(self.scan.header.stamp))
                if report is not None:
                    self.pub_victimas.publish(armar_reporte(*report, self.get_clock().now().to_msg()))
                    elapsed = self.now_s()-self.start_time
                    self.get_logger().info(f'REPORTE ÚNICO {self.victims.count}/{self.victims.limit}: {report}, t={elapsed:.1f}s (puntaje sin confirmar)')
                    if self.victims.count >= self.victims.limit:
                        self.returning = True
                        self.goal = None
        except (TransformException, ValueError, cv2.error) as exc:
            self.notice(f'Percepción esperando sincronización/TF: {exc}')

    def begin_sweep(self, angle):
        self.sweeping, self.sweep, self.sweep_last = True, 0., angle
        self.pause_until = self.now_s()+.8

    def plan(self, pose):
        start = self.grid.cell(*pose[:2])
        costs, parents = self.grid.paths(start)
        if not costs:
            self.goal, self.route = None, []
            self.notice('Sin ruta con margen de seguridad: robot detenido.')
            return
        if self.returning:
            home_cell = self.grid.cell(*self.home)
            destination = home_cell if home_cell in costs else None
        else:
            remaining = [c for c in self.targets if c not in self.covered and c in costs]
            destination = min(remaining,key=lambda c: costs[c]) if remaining else None
            if destination is None:
                self.returning = True
                home_cell = self.grid.cell(*self.home)
                destination = home_cell if home_cell in costs else None
        self.goal = destination
        self.route = self.grid.route(start,destination,parents) if destination is not None else []

    def control(self):
        if self.grid is None:
            return
        if self.finished:
            self.send()
            return
        if self.scan is None or time.monotonic()-self.scan_wall > self.p('timeout_sensor'):
            self.send()
            self.notice('Freno: LiDAR ausente o desactualizado.')
            return
        if time.monotonic()-self.image_wall > 1.0 or self.info is None:
            self.send()
            self.notice('Freno: falta cámara/calibración.')
            return
        try:
            # Scan-time pose avoids using a stale/latest map transform silently.
            t = self.tf.lookup_transform('map', self.p('base_frame'), Time.from_msg(self.scan.header.stamp))
            pose = (t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation))
            points = self.scan_points(self.p('base_frame'))
        except TransformException:
            self.send()
            self.notice('Esperando localización map → base_link y TF de sensores.')
            return
        if not self.localized():
            self.send()
            self.notice('Freno: localización incierta; comprobar AMCL y mapa.')
            return
        if self.home is None:
            self.home = pose[:2]
            self.begin_sweep(pose[2])
        now = self.now_s()
        if now-self.start_time > self.p('regreso_segundos') and not self.returning:
            self.returning, self.goal = True, None
        # All-around footprint checks also protect the swept volume during turns.
        if len(points) < 6:
            self.send()
            self.notice('Freno: escaneo sin suficientes retornos válidos.')
            return
        distance = np.linalg.norm(points[:,:2],axis=1)
        radius = self.p('radio_robot')
        margin = self.p('margen_seguridad')
        if np.min(distance) < radius+margin:
            self.send()
            self.notice('Freno: obstáculo dentro del margen del chasis.')
            return
        # Missing/NaN forward beams cannot be interpreted as a clear corridor.
        ranges = np.array(self.scan.ranges)
        angles = self.scan.angle_min + np.arange(len(ranges))*self.scan.angle_increment
        sector = np.abs(np.arctan2(np.sin(angles),np.cos(angles))) < .6
        usable = (np.isfinite(ranges)&(ranges>=self.scan.range_min)&(ranges<=self.scan.range_max)) | np.isposinf(ranges)
        if (sector.sum() < 3 or np.mean(usable[sector]) < .9 or
                abs(self.now_s()-seconds(self.scan.header.stamp)) > self.p('timeout_sensor') or
                abs(self.scan.angle_increment)*len(ranges) < 6.0):
            self.send()
            self.notice('Freno: cobertura o fecha del LiDAR no válida.')
            return
        if self.returning and math.dist(pose[:2],self.home) < .12:
            self.send()
            self.home_ticks += 1
            if self.home_ticks >= 15:
                self.pub_done.publish(Empty())
                self.finished = True
                self.get_logger().info('REGRESO ESTIMADO confirmado: /done enviado una vez. Verificar con juez.')
            return
        self.home_ticks = 0
        if now < self.observe_until and not self.returning:
            self.send()
            return
        if self.sweeping and not self.returning:
            delta = wrap(pose[2]-self.sweep_last)
            self.sweep += max(0.,delta)
            self.sweep_last = pose[2]
            if self.sweep >= 2*math.pi-.08:
                self.sweeping = False
                if self.goal is not None:
                    self.covered.add(self.goal)
                self.goal = None
            elif now < self.pause_until:
                self.send()
                return
            else:
                # Alternate small rotations and stationary observations.
                phase = now % 1.5
                self.send(0., .45 if phase < .7 else 0.)
                return
        if now-self.last_plan > 1.0 or self.goal is None:
            # Reuse already base-transformed returns for a dynamic obstacle layer.
            ca, sa = math.cos(pose[2]), math.sin(pose[2])
            for x,y,_ in points[::2]:
                self.grid.add_obstacle((pose[0]+ca*x-sa*y,pose[1]+sa*x+ca*y),now+4.)
            self.grid.refresh(now)
            self.plan(pose)
            self.last_plan = now
        if not self.route:
            self.send()
            return
        if not self.returning and math.dist(pose[:2],self.grid.point(self.goal)) < self.p('tolerancia_meta'):
            if not self.returning:
                self.begin_sweep(pose[2])
            self.send()
            return
        # Follow adjacent grid path points; never cut a corner with long lookahead.
        nearest = min(range(len(self.route)), key=lambda i: math.dist(pose[:2],self.route[i]))
        index = min(len(self.route)-1,nearest+1)
        target = self.home if self.returning and index == len(self.route)-1 else self.route[index]
        error = wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
        w = float(np.clip(1.8*error,-self.p('velocidad_angular_max'),self.p('velocidad_angular_max')))
        if abs(error) > .30:
            self.send(0.,w)
            return
        front = points[(points[:,0]>0)&(np.abs(points[:,1])<radius+margin)]
        clearance = float(np.min(front[:,0])-radius) if len(front) else float(np.min(distance)-radius)
        v = speed_limit(clearance,self.p('velocidad_lineal_max'),margin,self.p('desaceleracion'),self.p('timeout_sensor'))
        v = min(v, 1.2*math.dist(pose[:2],target))
        v *= max(0.,math.cos(error))
        self.send(v,w)


def main(args=None):
    rclpy.init(args=args)
    node = NodoEquipo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.grid is not None:
            node.send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
