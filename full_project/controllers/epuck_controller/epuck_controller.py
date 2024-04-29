# coding = utf-8
from deepbots.robots.controllers.csv_robot import CSVRobot
import sys
sys.path.append('..')
import numpy as np
from supervisor_controller.config import get_config

class Epuck2Robot(CSVRobot):
    def __init__(self):
        super().__init__(timestep=100)
        args = get_config().parse_known_args()[0]
        self.ps_sensor = []
        for i in range(8):
            self.ps_sensor.append(self.getDevice(f"ps{i}"))
            self.ps_sensor[i].enable(self.timestep)

        self.accelerometer = self.getDevice("accelerometer")
        self.accelerometer.enable(self.timestep)

        self.wheels = []
        for wheel_name in ['left wheel motor', 'right wheel motor']:
            wheel = self.getDevice(wheel_name)  # Get the wheel handle
            wheel.setPosition(float('inf'))  # Set starting position
            wheel.setVelocity(0.0)  # Zero out starting velocity
            self.wheels.append(wheel)

        self.weights = [[-1.3, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
         [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]]
        self.max_speed = 6.28
        self.robot_name = self.getName()[-1]
        self.timestep = args.timestep
        self.interval = args.interval
        self.num_agents = args.num_agents
        self.receiver_arb = self.initialize_receiver()


    def initialize_receiver(self):
        """
        初始化RAB信息的接收器

        :return: 接收器
        """
        receiver = self.getDevice('receiver01')
        receiver.enable(self.timestep//self.interval)
        return receiver

    def handle_receiver_arb(self):
        """
        接收RAB消息

        :return: RAB消息
        """
        message = 0
        if self.receiver_arb.getQueueLength() > 0:
            str_message = self.receiver_arb.getString()
            message = np.array(str_message.split(","),dtype=np.float32).reshape(self.num_agents,-1)
            self.receiver_arb.nextPacket()
        return message

    def run(self):
        """
        更新智能体在模拟环境中的状态。接收和发送对应信息，并设置信息发送/接收频率
        """
        i = 0
        while self.step(self.timestep//self.interval) != -1:
            self.handle_receiver_arb()
            if i%self.interval == 0:
                self.handle_receiver()
                self.handle_emitter()
                i = 0
            i += 1


    def create_message(self):
        """
        创建智能体需要发送的信息：8个红外传感器的值

        :return: 8个红外传感器的值
        """
        message = []
        message.append('a'+self.getName()[-1])
        #message.append(self.accelerometer.getValues())
        for rangefinder in self.ps_sensor:
            message.append(rangefinder.getValue())
        return message

    def use_message_data(self, message):
        """
        接收动作，并执行对应动作且进行避障

        :message: 需要执行的动作
        """
        action = int(message[int(self.getName()[-1])-1])  # Convert the string message into an action integer
        speed = np.zeros(2)
        # vel_actions = [[0,-1.57],[0,-0.785],[0,0],[0,0.785],[0,1.57],[0.1,-1.57],[0.1,-0.785],[0.1,0],[0.1,0.785],[0.1,1.57],[0,0]]
        # linear_vel = vel_actions[action][0]
        # angle_vel = vel_actions[action][1]
        # speed[0] = ((2 * linear_vel) - (angle_vel * 0.053)) / (2 * 0.0205)
        # speed[1] = ((2 * linear_vel) + (angle_vel * 0.053)) / (2 * 0.0205)
        if action == 0:
            speed[0] = 1 * self.max_speed
            speed[1] = 0.2 * self.max_speed
        elif action == 1:
            speed[0] = 0.2 * self.max_speed
            speed[1] = 1 * self.max_speed
        elif action == 2:
            speed[0] = self.max_speed
            speed[1] = self.max_speed
        elif action == 3:
            speed[0] = 0.5 * self.max_speed
            speed[1] = -0.5 * self.max_speed
        elif action == 4:
            speed[0] = -0.5 * self.max_speed
            speed[1] = 0.5 * self.max_speed
        else:
            speed[0] = 0
            speed[1] = 0
        #根据红外传感器的值进行避障
        right_obstacle = ((self.ps_sensor[0].getValue() > 80.0) | (self.ps_sensor[1].getValue() > 80.0) | (self.ps_sensor[2].getValue() > 80.0))
        left_obstacle = ((self.ps_sensor[5].getValue() > 80.0) | (self.ps_sensor[6].getValue() > 80.0) | (self.ps_sensor[7].getValue() > 80.0))
        if left_obstacle:
            speed[0] = 0.5 * self.max_speed
            speed[1] = -0.5 * self.max_speed
        elif right_obstacle:
            speed[0] = -0.5 * self.max_speed
            speed[1] = 0.5 * self.max_speed
        speed = np.clip(speed, -self.max_speed, self.max_speed)

        for i in range(len(self.wheels)):
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(speed[i])


robot_controller = Epuck2Robot()
robot_controller.run()
