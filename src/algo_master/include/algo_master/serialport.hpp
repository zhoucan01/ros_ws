#pragma once

#include "circular_buffer.hpp"

#include <cstdio>
#include <string>
#include <cstring>
#include <array>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/bin_to_hex.h>
#include <atomic>

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


const std::vector<std::string> kAlternativePorts = {"/dev/ttyACM0"};
std::string Portname;
#pragma pack(push, 1)
struct NavigationPLCSendMsg
{
    uint8_t m_FrameHead = 0xA5;
    float linear_vel_x;
    float linear_vel_y;
    float control_yaw_diff;
    float cur_x;
    float cur_y;
    bool has_path_;
    bool get_goal;
    uint8_t arrive_flag;  
    uint8_t close_flag;
    uint8_t need_tunnel;
    float tunnel_yaw_error;
    uint8_t if_on_attack;//是否正在追击
    uint8_t sentry_attitude_switch;//姿态切换
    float current_map_yaw;
    uint8_t seq;
    uint8_t m_FrameTail = 0xAA;
};
#pragma pack(pop)

constexpr uint8_t kNavFrameMsgId = 0x01;
constexpr uint8_t kDecisionFrameMsgId = 0x02;

#pragma pack(push, 1)
struct NavSerialMsg
{
    uint8_t m_FrameHead = 0xA5;
    uint8_t msg_id = kNavFrameMsgId;
    struct Target
    {
        float x{};
        float y{};
    } target_msg;

    struct ImuData
    {
        float m_ImuYaw{};
        float m_ImuPitch{};
    } imu_msg;

     struct EnemyPos {
         uint8_t If_vision_on;
         uint8_t Armor_id;
         int16_t Enemy_x;
         int16_t Enemy_y;
     } Enemy_Pos;
 
     /* Wheel odometry + gimbal yaw (for transforming chassis → IMU frame) */
     int16_t vx_wheel;        // chassis vx * 10000 (mm/s)
     int16_t vy_wheel;        // chassis vy * 10000 (mm/s)
     float   gimbal_yaw;      // gimbal yaw relative to chassis (rad)



    uint8_t m_FrameTail = 0xAA;
};

struct DecisionSerialMsg
{
    uint8_t m_FrameHead = 0xA5;
    uint8_t msg_id = kDecisionFrameMsgId;

    struct DecisionData
    {
        u_int8_t Decision_data_1;
        u_int8_t Decision_data_2;
        u_int8_t Decision_data_3;
        u_int8_t Decision_data_4;
        int16_t game_remain_time;
        uint8_t game_state;
    } Decision_Data;

    struct RefereeRawData
    {
        uint16_t projectile_allowance_17mm{};
        uint16_t current_hp{};
        uint16_t my_base_hp{};
        uint16_t we_outpost_hp{};
        uint16_t enemy_outpost_hp{};
        int16_t enemy_hero_x{};
        int16_t enemy_hero_y{};
        uint8_t real_sentry_attitude_switch{};
        uint8_t remaining_energy_flags{};
    } Referee_Raw_Data;

        struct Get_decision_Msg//决策端是否收到这些决策源的数据
    {
        uint8_t if_get_manual_msg;
        uint8_t if_get_radar_msg;//
    }Decision_Update_data;

    uint8_t m_FrameTail = 0xAA;
};

#pragma pack(pop)

constexpr size_t kSendBufSize = sizeof(NavigationPLCSendMsg);
constexpr size_t kNavRecvMsgSize = sizeof(NavSerialMsg);
constexpr size_t kDecisionRecvMsgSize = sizeof(DecisionSerialMsg);
constexpr size_t kRecvBufSize = 256;

//未经处理的256个uint8数组
using RawBufRecv = std::array<uint8_t, kRecvBufSize>;

//环形缓冲区并规定大小为1
CircularBuffer<RawBufRecv> msgRawBufs(1);
CircularBuffer<NavSerialMsg> msgSerialNavRecv(1);
CircularBuffer<DecisionSerialMsg> msgSerialDecisionRecv(1);

class NautilusSerialPort
{
public:
    NautilusSerialPort() { port_available = true; }
    ~NautilusSerialPort() {}
    /**
     *	@Brief:		打开串口,成功返回true，失败返回false
     *	@Param		portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等
     *	@Param		baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200
     *	@Param		parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验（仅适用于windows)
     *	@Param		databit(数据位): 4-8(windows),5-8(linux)，通常为8位
     *	@Param		stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位
     *	@Param		synchronizeflag(同步、异步,仅适用与windows): 0为异步，1为同步
     *	@Return:	真或假
     */
    bool OpenPort(const std::string &portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag);

    void ClosePort() { m_Close(); }

    bool Send(const NavigationPLCSendMsg &payload);

    // 检查接收帧中下标范围为[begin, end)的部分是否符合MsgType协议
    template <typename MsgType>
    bool VerifyFrame(const RawBufRecv &buf, size_t begin, size_t end, uint8_t expected_msg_id);

    void ReadRawBuf();

    void ProcRawBuf();

    void CheckAndReconnect(); // 如果连续timeout ms没有收到任何消息，尝试连接其他预设的串口

private:
    std::string m_Portname;
    int m_Baudrate;
    char m_Parity;
    char m_Databit;
    char m_Stopbit;
    char m_Synchronizeflag = 1;

    std::chrono::steady_clock::time_point last_received; // 最后一次收到消息的时间
    // 串口状态 原子操作保证多线程的数据安全
    std::atomic_bool port_available;

    int pHandle[16];
    char synchronizeflag;
    uint8_t sendFrame[kSendBufSize];
    // uint8_t recvFrame[kRecvBufSize];

    bool m_Open(const char *portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag);

    // 关闭串口，参数待定
    void m_Close();

    // 发送数据或写数据，成功返回发送数据长度，失败返回0
    int m_Send(const void *buf, int len);

    // 接受数据或读数据，成功返回读取实际数据的长度，失败返回0
    int m_Receive(void *buf, int maxlen);
};

bool NautilusSerialPort::m_Open(const char *portname,
                                int baudrate,
                                char parity,
                                char databit,
                                char stopbit,
                                char synchronizeflag)
{
    // (void)synchronizeflag;  // 显式标记未使用
    if (synchronizeflag)
    {
        // Add synchronization-specific logic here
        RCLCPP_INFO(rclcpp::get_logger("NautilusSerialPort"), "Synchronization enabled");
    }
    // 打开串口
    pHandle[0] = -1;
    // 以 读写、不阻塞 方式打开
    pHandle[0] = ::open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

    // 打开失败，则打印失败信息，返回false
    if (pHandle[0] == -1)
    {
        spdlog::critical("{} open failed , may be you need 'sudo' permission.", portname);
        return false;
    }

    // 设置串口参数
    // 创建串口参数对象
    struct termios options;
    // 先获得串口的当前参数
    if (tcgetattr(pHandle[0], &options) < 0)
    {
        spdlog::critical("{} open failed , get serial port attributes failed.", portname);
        return false;
    }

    // 设置波特率
    switch (baudrate)
    {
    case 4800:
        cfsetispeed(&options, B4800);
        cfsetospeed(&options, B4800);
        break;
    case 9600:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;
    case 19200:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;
    case 38400:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;
    case 57600:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;
    case 115200:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;
    default:
        spdlog::critical("{} open failed , unkown baudrate , only support 4800,9600,19200,38400,57600,115200.", portname);
        return false;
    }

    // 设置校验位
    switch (parity)
    {
        // 无校验
    case 0:
        options.c_cflag &= ~PARENB; // PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag &= ~INPCK;  // INPCK：使奇偶校验起作用
        break;
        // 设置奇校验
    case 1:
        options.c_cflag |= PARENB; // PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag |= PARODD; // PARODD：若设置则为奇校验,否则为偶校验
        options.c_cflag |= INPCK;  // INPCK：使奇偶校验起作用
        options.c_cflag |= ISTRIP; // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
        // 设置偶校验
    case 2:
        options.c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag &= ~PARODD; // PARODD：若设置则为奇校验,否则为偶校验
        options.c_cflag |= INPCK;   // INPCK：使奇偶校验起作用
        options.c_cflag |= ISTRIP;  // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
    default:
        spdlog::critical("{} open failed , unkown parity.", portname);
        return false;
    }

    // 设置数据位
    switch (databit)
    {
    case 5:
        options.c_cflag &= ~CSIZE; // 屏蔽其它标志位
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag &= ~CSIZE; // 屏蔽其它标志位
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag &= ~CSIZE; // 屏蔽其它标志位
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag &= ~CSIZE; // 屏蔽其它标志位
        options.c_cflag |= CS8;
        break;
    default:
        spdlog::critical("{} open failed , unkown databit.", portname);
        return false;
    }

    // 显式启用本地连接和接收器，避免某些设备默认不接收数据。
    options.c_cflag |= (CLOCAL | CREAD);

    // 设置停止位
    switch (stopbit)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; // CSTOPB：使用1位停止位
        break;
    case 2:
        options.c_cflag |= CSTOPB; // CSTOPB：使用2位停止位
        break;
    default:
        spdlog::critical("{} open failed , unkown stopbit.", portname);
        return false;
    }

    // 设置原始输入模式
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_iflag &= ~(ICRNL | IGNCR);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    spdlog::info(
        "Opening serial port {} with {}-{}-{}-{}",
        portname, baudrate, static_cast<int>(databit), static_cast<int>(parity), static_cast<int>(stopbit));

    // 激活新配置
    if ((tcsetattr(pHandle[0], TCSANOW, &options)) != 0)
    {
        spdlog::critical("{} open failed , can not complete set attributes.", portname);
        return false;
    }

    return true;
}

void NautilusSerialPort::m_Close()
{
    if (pHandle[0] != -1)
    {
        ::close(pHandle[0]);
    }
}

int NautilusSerialPort::m_Send(const void *buf, int len)
{
    int sendCount = 0;
    if (pHandle[0] != -1)
    {
        // 将 buf 和 len 转换成api要求的格式
        const char *buffer = (char *)buf;
        size_t length = len;
        // 已写入的数据个数
        ssize_t tmp;

        while (length > 0)
        {
            if ((tmp = write(pHandle[0], buffer, length)) <= 0)
            {
                if (tmp < 0 && errno == EINTR)
                {
                    tmp = 0;
                }
                else
                {
                    break;
                }
            }
            length -= tmp;
            buffer += tmp;
        }

        sendCount = len - length;
    }

    return sendCount;
}
/// @brief 使用：：read函数从phandle中读取maxlen个数据到buf中
/// @param buf
/// @param maxlen
/// @return 读取到的字节数
int NautilusSerialPort::m_Receive(void *buf, int maxlen)
{
    int receiveCount = ::read(pHandle[0], buf, maxlen);
    if (receiveCount < 0)
    {
        receiveCount = 0;
    }
    return receiveCount;
}

bool NautilusSerialPort::OpenPort(const std::string &portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag)
{
    if (m_Open(portname.c_str(), baudrate, parity, databit, stopbit, synchronizeflag))
    {
        m_Portname = portname;
        m_Baudrate = baudrate;
        m_Parity = parity;
        m_Databit = databit;
        m_Stopbit = stopbit;
        m_Synchronizeflag = synchronizeflag;

        return true;
    }
    else
    {
        return false;
    }
}
/// @brief 从串口中读取数据并缓存到缓存区
void NautilusSerialPort::ReadRawBuf()
{
    RawBufRecv rawBuf;
    bool first_frame_logged = false;
    while (true)
    {
        rawBuf.fill(0); ////将所有的元素填充为0
        if (!port_available)
            continue;
        ////array的成员函数data返回数组首个元素的指针
        const int received_len = m_Receive(rawBuf.data(), rawBuf.size());
        if (received_len > 0)
        {
            if (!first_frame_logged)
            {
                spdlog::info("First serial read received {} bytes from {}", received_len, m_Portname);
                first_frame_logged = true;
            }
            msgRawBufs.Push(rawBuf);
            ////最后一次压入缓存区的时间戳
            last_received = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
/// @brief 检验数据位是否正确
/// @tparam MsgType 自定义数据类型
/// @param buf
/// @param begin
/// @param end
/// @return
template <typename MsgType>
bool NautilusSerialPort::VerifyFrame(const RawBufRecv &buf, size_t begin, size_t end, uint8_t expected_msg_id)
{
    if (end - begin != sizeof(MsgType) || buf.size() < sizeof(MsgType) || end > buf.size())
        return false;

    if (buf[begin] != 0xA5 || buf[end - 1] != 0xAA)
        return false;

    return buf[begin + 1] == expected_msg_id;
}

void NautilusSerialPort::ProcRawBuf()
{
    NavSerialMsg nav_payload;
    DecisionSerialMsg decision_payload;
    RawBufRecv rawBuf;
    rawBuf.fill(0);
    while (true)
    {
        if (msgRawBufs.Pop(rawBuf))
        {
            for (size_t i = 0; i < rawBuf.size();)
            {
                if (rawBuf[i] != 0xA5) {
                    ++i;
                    continue;
                }

                if (i + 2 > rawBuf.size()) {
                    break;
                }

                const uint8_t msg_id = rawBuf[i + 1];
                if (msg_id == kNavFrameMsgId) {
                    const size_t j = i + kNavRecvMsgSize;
                    if (j <= rawBuf.size() &&
                        VerifyFrame<NavSerialMsg>(rawBuf, i, j, kNavFrameMsgId)) {
#ifdef SHOW_SERIALPORT_MSG
                        spdlog::info("recv nav buf: {:X:n}", spdlog::to_hex(rawBuf.begin() + i, rawBuf.begin() + j));
#endif // SHOW_SERIALPORT_MSG
                        std::memcpy(&nav_payload, rawBuf.data() + i, kNavRecvMsgSize);
                        msgSerialNavRecv.Push(nav_payload);
                        i = j;
                        continue;
                    }
                } else if (msg_id == kDecisionFrameMsgId) {
                    const size_t j = i + kDecisionRecvMsgSize;
                    if (j <= rawBuf.size() &&
                        VerifyFrame<DecisionSerialMsg>(rawBuf, i, j, kDecisionFrameMsgId)) {
#ifdef SHOW_SERIALPORT_MSG
                        spdlog::info("recv decision buf: {:X:n}", spdlog::to_hex(rawBuf.begin() + i, rawBuf.begin() + j));
#endif // SHOW_SERIALPORT_MSG
                        std::memcpy(&decision_payload, rawBuf.data() + i, kDecisionRecvMsgSize);
                        msgSerialDecisionRecv.Push(decision_payload);
                        i = j;
                        continue;
                    }
                }

                ++i;
            }
        }
    }
}
/// @brief 串口发送
/// @param payload 发送的数据
/// @return
bool NautilusSerialPort::Send(const NavigationPLCSendMsg &payload)
{
    memcpy(sendFrame, &payload, kSendBufSize);

#ifdef SHOW_SERIALPORT_MSG
    spdlog::info("send buf: {:X:n}", spdlog::to_hex(sendFrame, sendFrame + kSendBufSize));
#endif // SHOW_SERIALPORT_MSG

    if (!port_available)
        return 0;

    return m_Send(sendFrame, sizeof(sendFrame)) == kSendBufSize;
}
// /// @brief 检查串口并且控制串口是否可读写
// void NautilusSerialPort::CheckAndReconnect()
// {
//     static constexpr long long timeout = 1000;
//     rclcpp::executors::SingleThreadedExecutor executor;  // 创建Executor
//     auto node = std::make_shared<rclcpp::Node>("serial_port_node");
//     executor.add_node(node); 
//     while (true)
//     {
//         if (last_received == std::chrono::steady_clock::time_point())
//             continue;

//         auto now = std::chrono::steady_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_received);

//         if (duration.count() >= timeout)
//         {
//             spdlog::error("Connection on {} failed, trying other serialports", m_Portname);
//             size_t idx = 0;
//             // ros::Rate rate(1);
//             rclcpp::Rate rate(1);
//             while (true)
//             {
//                 std::string alternative_port = kAlternativePorts[idx];
//                 idx = (idx + 1) % kAlternativePorts.size();

//                 port_available = false;

//                 ClosePort();
//                 if (OpenPort(alternative_port, m_Baudrate, m_Parity, m_Databit, m_Stopbit, m_Synchronizeflag))
//                 {
//                     spdlog::info("Connected to {}", Portname);
//                     port_available = true;
//                     break;
//                 }
//                 executor.spin_some();  // 替换ros::spinOnce()
//                 rate.sleep();
//             }
//         }
//     }
// }

// @brief 检查串口并且控制串口是否可读写
void NautilusSerialPort::CheckAndReconnect()
{
    static constexpr long long timeout = 1000;  // 超时时间（毫秒）
    bool waiting_for_first_frame_logged = false;

    spdlog::info("CheckAndReconnect thread started for initial port {}", m_Portname);

    while (true)
    {
        // 如果 last_received 没有被初始化，则跳过检查
        if (last_received == std::chrono::steady_clock::time_point()) {
            if (!waiting_for_first_frame_logged) {
                spdlog::info("CheckAndReconnect waiting for first received frame before timeout monitoring starts");
                waiting_for_first_frame_logged = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        waiting_for_first_frame_logged = false;

        // 计算从上次接收到数据到现在的时间间隔
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_received);

        // 如果超时未接收到数据，则尝试重新连接
        if (duration.count() >= timeout)
        {
            spdlog::warn(
                "No serial frame received from {} for {} ms, starting reconnect attempts",
                m_Portname, duration.count());

            size_t idx = 0;
            while (true)
            {
                // 获取替代串口名称
                std::string alternative_port = kAlternativePorts[idx];
                idx = (idx + 1) % kAlternativePorts.size();
                spdlog::info("Trying alternative serial port {}", alternative_port);

                port_available = false;

                // 关闭当前串口并尝试打开替代串口
                ClosePort();
                if (OpenPort(alternative_port, m_Baudrate, m_Parity, m_Databit, m_Stopbit, m_Synchronizeflag))
                {
                    spdlog::info(
                        "Reconnected serial port on {} after timeout/reflash recovery attempt",
                        m_Portname);
                    port_available = true;
                    last_received = std::chrono::steady_clock::now();
                    break;
                }

                // 等待一段时间
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
