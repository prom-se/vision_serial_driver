#include "../include/vision_serial_driver/vision_serial_driver_node.hpp"

serial_driver_node::serial_driver_node(std::string device_name,std::string node_name)
:rclcpp::Node(node_name),vArray{new visionArray},rArray{new robotArray},
dev_name{new std::string(device_name)},
portConfig{new SerialPortConfig(115200,FlowControl::NONE,Parity::NONE,StopBits::ONE)},ctx{IoContext(2)}
{
  RCLCPP_INFO(get_logger(), "节点:/%s启动",node_name.c_str());
  
  //清零
  memset(vArray->array,0,sizeof(visionArray));
  memset(rArray->array,0,sizeof(robotArray));

  //设置重启计时器1hz.
  reopenTimer=create_wall_timer(
    1s, std::bind(&serial_driver_node::serial_reopen_callback, this));
  
  //设置发布计时器500hz.
  publishTimer=create_wall_timer(
    2ms, std::bind(&serial_driver_node::robot_callback, this));

  //发布Robot信息.
  publisher=create_publisher<vision_interfaces::msg::Robot>(
    "/serial_driver/robot",10);

  //订阅AutoAim信息.
  subscription=create_subscription<vision_interfaces::msg::AutoAim>(
    "/serial_driver/aim_target",10,std::bind(&serial_driver_node::auto_aim_callback,this,std::placeholders::_1));
  
  //设置串口读取线程.
  serialReadThread=std::thread(&serial_driver_node::serial_read_thread,this);
};

serial_driver_node::~serial_driver_node(){
  if(serialReadThread.joinable()) {
    serialReadThread.join();
  }
  if(serialDriver.port()->is_open()) {
    serialDriver.port()->close();
  }
}

void serial_driver_node::serial_reopen_callback(){
  //串口失效时尝试重启
  if(!isOpen){
    try{
      RCLCPP_WARN(get_logger(),"重启串口:%s...",dev_name->c_str());
      serialDriver.init_port(*dev_name,*portConfig);
      serialDriver.port()->open();
      isOpen=serialDriver.port()->is_open();
    }
    catch(const std::system_error & error){
      RCLCPP_ERROR(get_logger(),"打开串口:%s失败",dev_name->c_str());
      isOpen=false;
    }
    if(isOpen)RCLCPP_INFO(get_logger(),"打开串口:%s成功",dev_name->c_str());
  }
}

void serial_driver_node::serial_read_thread(){
  while(rclcpp::ok()){
    std::vector<uint8_t> head(2);
    std::vector<uint8_t> robotData(sizeof(robotMsg)-2);
    if(isOpen){
      try{
        serialDriver.port()->receive(head);
        if(head[0]==0xA5 && head[1]==0x00){//包头为0xA5
          serialDriver.port()->receive(robotData);
          robotData.resize(sizeof(robotMsg));
          robotData.insert(robotData.begin(),head[1]);
          robotData.insert(robotData.begin(),head[0]);
          memcpy(rArray->array,robotData.data(),sizeof(robotMsg));
        }
      }
      catch(const std::exception &error){
        RCLCPP_ERROR(get_logger(),"读取串口时发生错误.");
        isOpen=false;
      }
    }
  }
}

void serial_driver_node::serial_write(){
  std::vector<uint8_t> visionData(sizeof(visionMsg));
  memcpy(visionData.data(),vArray->array,sizeof(visionMsg));
  try{
    serialDriver.port()->send(visionData);
  }
  catch(const std::exception &error){
    RCLCPP_ERROR(get_logger(),"写入串口时发生错误.");
    isOpen=false;
  }
}  

void serial_driver_node::auto_aim_callback(const vision_interfaces::msg::AutoAim vMsg){
  if(isOpen){
    vArray->msg.head=0xA5;
    vArray->msg.aimPitch=vMsg.aim_pitch;
    vArray->msg.aimYaw=vMsg.aim_yaw;
    serial_write();
  }
}

void serial_driver_node::robot_callback(){
  if(isOpen){
    auto msg=vision_interfaces::msg::Robot();
    msg.mode=rArray->msg.mode;
    msg.foe_color=rArray->msg.foeColor;
    msg.self_yaw=rArray->msg.robotYaw;
    msg.self_pitch=rArray->msg.robotPitch;
    msg.muzzle_speed=rArray->msg.muzzleSpeed!=0?rArray->msg.muzzleSpeed:25.0;
    publisher->publish(msg);
  }
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  std::string dev_name = argc!=1?argv[1]:"/dev/ttyACM0";
  std::shared_ptr<serial_driver_node> node= std::make_shared<serial_driver_node>(dev_name,"vision_serial_driver");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}