
#include "routing/routing_path.h"
#include "common/pnc_point.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

const std::vector<MapPoint> RoutingPath::routing_path_points() const {
  return routing_path_points_;
}

void RoutingPath::set_routing_path(
    const std::vector<MapPoint> routing_path_point) {
  routing_path_points_ = routing_path_point;
}

//从csv文件中初始化routing_path
std::vector<MapPoint> RoutingPath::GetRoutingPathFromCSV() {
  //读取csv文件

  std::ifstream fp(
      ".\\file\\routing_path.csv"); //定义声明一个ifstream对象，指定文件路径

  if (!fp.is_open()) {
    std::cout << "Error: opening file fail" << std::endl;
    std::exit(1);
  }
  std::string line;

  std::istringstream sin; //将整行字符串line读入到字符串istringstream中
  std::vector<std::string> words; //声明一个字符串向量
  std::string word;

  std::getline(fp, line); // 读取标题行
  std::vector<MapPoint> path;
  while (std::getline(fp, line)) // 从文件fp读取一行数据至line
  {
    sin.clear();
    sin.str(line);
    words.clear();
    int col = 1;
    MapPoint point;
    while (std::getline(
        sin, word,
        ',')) //将字符串流sin中的字符读到field字符串中，以逗号为分隔符
    {
      if (col == 1) {
        point.x = stod(word);
      } else {
        point.y = stod(word);
      }
      std::cout << word;
      col++;
    }
    path.push_back(point);
  }
  fp.close();

  return path;
}

void RoutingPath::CreatePath() {
  routing_path_points_.clear();
  //第一段
  for (int i = 0; i < 2000; i++) {
    MapPoint point;
    point.x = i * 0.1;
    point.y = 0;
    routing_path_points_.push_back(point);
  }

  //第二段
  for (int i = 0; i < 200; i++) {
    MapPoint point;
    point.x = 200;
    point.y = i * 0.1;
    routing_path_points_.push_back(point);
  }

  //第三段
  for (int i = 0; i < 800; i++) {
    MapPoint point;
    point.x = 200 + 0.1 * i;
    point.y = 20;
    routing_path_points_.push_back(point);
  }
}

/*文件路径的表示
绝对路径
例如：
pDummyFile = fopen（"D:\\vctest\\glTexture\\texture\\dummy.bmp", "rb"）；

//给出了从盘符开始的全部路径，这里需要注意的是“\”要用双斜线"\\"或者反斜线“/”
vc工程默认访问的目录是工程目录（即c++文件所在目录）， 相对路径有以下多种形式：

pDummyFile = fopen（"dummy.bmp", "rb"）；
//bmp文件就在vc工程目录下，**和dsw文件同属一个目录**。 pDummyFile =
fopen（"..\\texture\\dummy.bmp", "rb"）；
//表示bmp文件在工程目录的同级目录texture中，因此路径是先退出工程目录再进入texture目录访问到bmp
//文件。  **“..”表示退到当前目录的上一级目录（父目录）** pDummyFile =
fopen（".\\texture\\dummy.bmp", "rb"）；
//表示bmp文件就在工程目录的子目录texture中。
// **“.”表示当前默认目录，即工程目录，然后在进入其子目录texture访问到文件.
注意，对相对路径而言，路径表示中的“\”也要用双斜线"\\"或者反斜线“/”**

*/