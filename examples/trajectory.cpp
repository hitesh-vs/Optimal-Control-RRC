#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("effort_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;
  double values[] = {0.0,0.2320191772262227,0.4600111230308208,0.6846907222175552,0.9067728595901872,1.126972419952477,1.3460042881081875,1.5645833488610783,1.7834244870149107,2.003242587373446,2.2247525347404453,2.4486692139196697,2.67570750971488,2.9065823069298378,3.141821037287565,3.380225606481631,3.619667978288165,3.858011102961755,4.093117930756993,4.322851411928466,4.545074481342144,4.757629653161856,4.958298764212666,5.144852433013796,5.315061278084468,5.466695917943897,5.597526971111309,5.705371719480938,5.789309396744905,5.849795874180783,5.887356678016615,5.902517334480444,5.895803369800308,5.867740310204252,5.818909822059132,5.750266791374653,5.662918356216984,5.557972103773414,5.43653562123122,5.299716495777687,5.148622942669047,4.984496327778733,4.808875093593084,4.62333787901118,4.429463322932099,4.228830064254923,4.023016741878728,3.81356941439516,3.601475964727767,3.3872553063798536,3.1714117835619406,2.9544497404845487,2.736873521358198,2.5191874703934105,2.3020605160246794,2.0869660940466237,1.8756230770776516,1.6697503977158528,1.4710669885593175,1.28129178220613,1.1021388048775786,0.9349424746049828,0.7803937471885974,0.6391212492715842,0.511753607497105,0.398919448508325,0.3012473989484089,0.219267468067292,0.1523632379186529,0.0991786601069732,0.0583453590625804,0.0284949592158012,0.0082590849969624,-0.0037306398333237,-0.0088625403359868,-0.0085971523278223,-0.0044113453218209,0.0022180111690271,0.009814047631731,0.0168998945533005,0.022019026424978,0.0245057909025561,0.0247219078555849,0.0230977581679003,0.020063722723338,0.0160501824057336,0.0114875180989231,0.0067875714818572,0.0022094929223738,-0.0020626580768103,-0.0058453446518608,-0.0089550299389429,-0.0112081770742219,-0.012421393112723,-0.012499093603783,-0.0115825125804759,-0.0098524077927377,-0.0074895369905041,-0.0046746579237111,-0.0015885283422946,0.0015880925207016,0.0046744113207498,0.0074896006018382,0.0098528314248473,0.0115832748506575,0.012500101940149,0.0124224837542024,0.0112091296548805,0.0089556548744182,0.0058455155408502,0.0020623117541774,-0.0022103563855995,-0.0067888887784798,-0.0114891628479806,-0.0160519884855117,-0.0200655317172012,-0.0230994251249868,-0.0247233012908061,-0.0245067927965969,-0.0220195322242969,-0.0168998145958817,-0.0098133284805649,-0.0022166520346662,0.0044132921588164,0.0085995815168859,0.0088652934565443,0.0037335053947941,-0.0082563537991311,-0.028492566001253,-0.0583434425800852,-0.0991772942336974,-0.152362431660158,-0.219267165557539,-0.3012474800035843,-0.3989197618565381,-0.5117540222844292,-0.6391216594926804,-0.7803940716867164,-0.934942657071962,-1.1021388138538373,-1.2812916102677054,-1.4710666439249596,-1.6697498992956623,-1.8756224544090967,-2.086965387294535,-2.30205977598125,-2.51918675847766,-2.736872907443434,-2.954449295777101,-3.17141157914429,-3.387255413210632,-3.601476453641773,-3.813570356103345,-4.023018206246848,-4.228832097084025,-4.4294659183222524,-4.623340977748374,-4.808878583149217,-4.984500042311613,-5.148626663022404,-5.2997199593715125,-5.436538595157675,-5.557974417558621,-5.662919901975571,-5.750267523809748,-5.818909758462369,-5.867739530409407,-5.8958020004688025,-5.902515510579938,-5.887354537026578,-5.849793556092491,-5.789307044061444,-5.705369477217201,-5.59752498554894,-5.466694312024497,-5.315060128852725,-5.144851770783416,-4.958298572566359,-4.757629868951347,-4.545074994688173,-4.322852075925648,-4.093118613965371,-3.858011707072604,-3.619668438137288,-3.380225890049376,-3.141821145698817,-2.9065822743213614,-2.675707389548312,-2.448669053584985,-2.2247523727970138,-2.00324245355003,-1.783424402209653,-1.564583325141515,-1.3460043287112442,-1.1269725192844684,-0.9067730032268208,-0.6846908869039192,-0.4600112766813965,-0.2320192789248863,0.0};

  commands.data.push_back(0);
  publisher->publish(commands);
  std::this_thread::sleep_for(0.022378639513832697s);

  for (int i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
    commands.data[0] = values[i+1];
    publisher->publish(commands);
    std::this_thread::sleep_for(0.022378639513832697s);
  }

  rclcpp::shutdown();

  return 0;
}
