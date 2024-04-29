from config import get_config
import numpy as np
from deepbots.supervisor.controllers.csv_supervisor_env import CSVSupervisorEnv
import math
import random
from controller import Supervisor
from scipy.spatial.transform import Rotation as R
from scipy import interpolate

class Epuck2Supervisor(CSVSupervisorEnv):
    def __init__(self,all_args=None):
        super().__init__(timestep=all_args.timestep)
        self.args = all_args
        self.num_agents = self.args.num_agents
        self.measure_radius = self.args.measure_radius
        self.episode_length = self.args.episode_length
        self.num_actions = self.args.num_actions
        self.timestep = self.args.timestep
        self.interval = self.args.interval
        self.need_real_message = self.args.need_real_message
        self.std_gaussian_noise = self.args.std_gaussian_noise
        self.need_real_range_noise = self.args.need_real_range_noise
        self.prob_measure_fail = self.args.prob_measure_fail
        self.radius_epuck = self.args.radius_epuck
        self.robot = []
        for i in range(1,self.num_agents+1):
            self.robot.append(self.getFromDef(f"epuck0{i}"))

        self.m_cMuValues = [(0, 7.646890), (2, 7.596525), (5, 7.249550),
                            (10, 7.084636), (15, 6.984497), (30, 6.917447),
                            (45, 6.823188), (60, 6.828551), (80, 6.828551)]

        self.m_cSigmaValues = [(0, 0.3570609), (2, 0.3192310), (5, 0.1926492),
                               (10, 0.1529397), (15, 0.1092330), (30, 0.1216533),
                               (45, 0.1531546), (60, 0.1418425), (80, 0.1418425)]

        self.m_fExpA = 9.06422181283387
        self.m_fExpB = -0.00565074879677167

        self.cleanup()
        self.emitter_arb = []
        for i in range(1,self.num_agents+1):
            self.emitter_arb.append(self.initialize_emitter(i))

    def initialize_emitter(self, id):
        """
        初始化测距板（range and bearing, RAB）信息的发射器

        :id: 发送给id对应的agent
        :return: id对应的发送器
        """
        emitter = self.getDevice('emitter0'+str(id))
        return emitter

    def Interpolate(self, Range, Values):
        """
        插值函数

        :Range: 智能体间的测距值
        :Values: 插值使用的数据点
        :return: 插值结果（模拟的测距传感器值）
        """
        Points, Values = zip(*Values)
        f = interpolate.interp1d(Points, Values, fill_value="extrapolate")
        return f(Range*100)
    def handle_emitter_arb(self):
        """
        模拟RAB传感器并发送RAB消息

        :return: 真实的距离、角度；模拟RAB的距离、角度
        """
        epuck_pos = np.zeros((self.num_agents, 1, 3), dtype=np.float32)
        rotation_vector = np.zeros((self.num_agents, 4), dtype=np.float32)
        for i in range(self.num_agents):
            epuck_pos[i] = self.robot[i].getField('translation').getSFVec3f()
            rotation_vector[i] = self.robot[i].getField('rotation').getSFRotation()
        CVectorRtoS = epuck_pos - epuck_pos.transpose(1,0,2)
        #计算旋转矩阵
        rotation = R.from_rotvec(rotation_vector[..., :3] * rotation_vector[..., -1][:, np.newaxis])
        rotation_matrix_inv = rotation.inv().as_matrix().astype(np.float32)
        if self.need_real_message:
            #旋转到接收者坐标系
            CVectorStoR_real = np.matmul(np.tile(rotation_matrix_inv, (self.num_agents, 1, 1, 1)), -CVectorRtoS[:, :, :, np.newaxis])
            #计算真实距离、角度
            dis_real = np.clip(np.linalg.norm(CVectorStoR_real,axis=2)[...,0]-self.radius_epuck*2, a_min=0.0,a_max=None)
            angle_real = np.arctan2(CVectorStoR_real[:, :, 1], CVectorStoR_real[:, :, 0])[..., 0]
            message_real = np.stack((dis_real, angle_real), axis=-1)
        #加入高斯噪声
        if self.std_gaussian_noise > 0:
            r = np.random.normal(loc=0,scale=0.01,size=(self.num_agents,self.num_agents))
            inclination = np.random.uniform(low=0,high=np.pi,size=(self.num_agents,self.num_agents))
            azimuth = np.random.uniform(low=0, high=np.pi*2, size=(self.num_agents, self.num_agents))
            CVectorRtoS[..., 0] += r * np.sin(inclination) * np.cos(azimuth)
            CVectorRtoS[..., 1] += r * np.sin(inclination) * np.sin(azimuth)
            CVectorRtoS[..., 2] += r * np.cos(inclination)

        CVectorStoR = -CVectorRtoS
        #旋转到接收者坐标系
        CVectorStoR_sim = np.matmul(np.tile(rotation_matrix_inv,(self.num_agents,1,1,1)),CVectorStoR[:,:,:,np.newaxis])
        dis_sim = np.clip(np.linalg.norm(CVectorStoR_sim,axis=2)[...,0]-self.radius_epuck*2, a_min=0.0,a_max=None)
        angle_sim = np.arctan2(CVectorStoR_sim[:,:,1],CVectorStoR_sim[:,:,0])[...,0]
        #模拟距离传感器噪声
        if self.need_real_range_noise:
            # fMu = torch.from_numpy(self.Interpolate(dis, self.m_cMuValues))
            # fSigma = torch.from_numpy(self.Interpolate(dis, self.m_cSigmaValues))
            # fPower = torch.exp(torch.distributions.Normal(fMu, fSigma).sample())
            # real_dis = np.clip(np.exp(self.m_fExpA + self.m_fExpB * fPower.numpy()), a_min=0.0,a_max=None)
            fMu = self.Interpolate(dis_sim, self.m_cMuValues)
            fSigma = self.Interpolate(dis_sim, self.m_cSigmaValues)
            fPower = np.zeros((self.num_agents,self.num_agents),dtype=np.float32)
            for i in range(self.num_agents):
                for j in range(self.num_agents):
                    fPower[i][j] = np.random.lognormal(fMu[i][j], fSigma[i][j])
            dis_sim = np.clip(np.exp(self.m_fExpA + self.m_fExpB * fPower), a_min=0.0, a_max=None) / 100

        message_sim = np.stack((dis_sim,angle_sim),axis=-1)
        #模拟测距范围
        receive_sign = np.eye(self.num_agents,dtype=np.bool_) | (dis_sim > self.measure_radius)
        #模拟测距失败概率
        if self.prob_measure_fail > 0:
            receive_sign |= (np.random.rand(self.num_agents,self.num_agents)<=self.prob_measure_fail)

        send_message = np.where(receive_sign[:, :, np.newaxis], np.zeros_like(message_sim), message_sim)
        #发送消息给所有agent
        for i in range(0, self.num_agents):
            send_mess_to_agent = send_message[:,i].reshape(-1)
            string_message = ",".join(map(str, send_mess_to_agent.tolist()))
            self.emitter_arb[i].send(string_message.encode("utf-8"))

        if self.need_real_message:
            refer_message = np.where(receive_sign[:, :, np.newaxis], np.zeros_like(message_real), message_real)
            return refer_message, send_message

        return None, send_message


    def step(self,action):
        """
        :action: 在每个时刻t，智能体执行对应动作
        :return: 真实的距离、角度；模拟RAB的距离、角度
        """
        real_dis = np.zeros((self.interval, self.num_agents, self.num_agents),dtype=np.float32)
        real_angle = np.zeros((self.interval, self.num_agents, self.num_agents), dtype=np.float32)
        rab_dis = np.zeros((self.interval, self.num_agents, self.num_agents), dtype=np.float32)
        rab_angle = np.zeros((self.interval, self.num_agents, self.num_agents), dtype=np.float32)
        for i in range(self.interval):
            real_message, sim_message = self.handle_emitter_arb()
            real_dis[i] = real_message[...,0]
            real_angle[i] = real_message[..., 0]
            rab_dis[i] = sim_message[..., 0]
            rab_angle[i] = sim_message[..., 0]
            if super(Supervisor, self).step(self.timestep//self.interval) == -1:
                exit()

        self.handle_emitter(action)

        return real_dis, real_angle, rab_dis, rab_angle
    def cleanup(self) -> None:
        """Prepares torch buffers for RL data collection."""
        self.extras = {}

    def get_default_observation(self):
        """
        初始化智能体的方向和位置
        """
        per_row = int(math.ceil(math.sqrt(self.num_agents)))
        x_spacing = 0.3
        z_spacing = 0.3
        #xmin = -0.5 * x_spacing * (per_row - 1)
        xmin = -0.5 * x_spacing * per_row
        zmin = -0.5 * z_spacing * (per_row - 2)
        count = 0
        # import pdb;pdb.set_trace()
        pos = []
        for j in range(per_row):
            agent_up = zmin + j * z_spacing
            for k in range(per_row):
                if count >= self.num_agents:
                    break
                agentx = xmin + k * x_spacing
                pos.append([agentx, agent_up, 0.0])

                count += 1
        agent_pos = []
        agent_pos.append(np.array(pos))

        agent_pos = np.array(agent_pos)
        for i in range(self.num_agents):
            epuck_default_pos = self.robot[i].getField('translation').getSFVec3f()
            robot_default_rotation = self.robot[i].getField('rotation').getSFRotation()
            epuck_default_pos[:2] = agent_pos[0][i][:2]
            robot_default_rotation[3] = random.uniform(-np.pi, np.pi)
            self.robot[i].getField('translation').setSFVec3f(epuck_default_pos)
            self.robot[i].getField('rotation').setSFRotation(robot_default_rotation)

        return None

    def handle_receiver(self):
        """
        接收每个智能体的8个红外传感器的信息

        :return: 8个红外传感器的信息
        """
        message = np.zeros((self.num_agents,8))

        for i in range(self.num_agents):
            if self.receiver.getQueueLength() > 0:
                try:
                    string_message = self.receiver.getString().split(',')
                except AttributeError:
                    string_message = self.receiver.getData().decode("utf-8")
                self.receiver.nextPacket()
                idx = int(string_message[0][1])-1
                message[idx] = np.array(string_message[1:]).astype(np.float32)
        return message

    def handle_emitter(self, action):
        """
        发送给每个智能体其需要执行的动作
        """
        message = (",".join(map(str, action))).encode("utf-8")
        self.emitter.send(message)

    def reset(self):
        """
        初始化模拟环境
        """
        self.simulationReset()
        self.simulationResetPhysics()
        super(Supervisor, self).step(self.timestep//self.interval)

        self.get_default_observation()
        for _ in range(self.interval-1):
            super(Supervisor, self).step(self.timestep//self.interval)
        self.handle_receiver()
        return None


if __name__ == "__main__":
    all_args = get_config().parse_known_args()[0]
    env = Epuck2Supervisor(all_args)
    episodes = all_args.episodes
    for episode in range(episodes):
        env.reset()
        env.step([5, 5, 5, 5])
        for step in range(all_args.episode_length):
            #采用随机动作
            actions = np.random.randint(0,all_args.num_actions,size=(all_args.num_agents,))
            real_dis, real_angle, rab_dis, rab_angle = env.step(actions.tolist())
            #打印出真实距离、角度和模拟RAB传感器的距离、角度
            print(real_dis)
            print(real_angle)
            print(rab_dis)
            print(rab_angle)

