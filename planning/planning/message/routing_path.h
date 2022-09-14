#include <vector>

/*
routing
全局的xy变量
*/
class MapPoint
{
public:
  /* data */
  double x;
  double y;
};

class RoutingPath
{
public:
  const std::vector<MapPoint> routing_path_points() const;

  void set_routing_path(const std::vector<MapPoint> routing_path_point);

  //从csv文件中初始化routing_path
  std::vector<MapPoint> GetRoutingPathFromCSV();

private:
  std::vector<MapPoint> routing_path_points_;
};

/*
1.成员变量设置为private,设置   成员变量(get_成员变量)  set_成员变量  public成员函数，用来读写成员变量，函数使用const类型
2.成员变量用小写，私有成员变量后面加_
*/

/*一、const修饰指针
int b = 500;

1、const int * a = & b;

2、int const * a = & b;

3、int * const a = & b;

4、const int * const a = & b;

对于1和2
const 放在*左侧，就是用来修饰指针所指向的变量，即指针指向的是常量。
若a是仓库管理员，b是仓库。即仓库中的货物（*a）不允许改变。但是可修改指针指向或b的值来改变*a。
对于3
const放在*的右侧,表示管理的仓库不可改变，只能是那一个，但是仓库中的货物（*a）可以改变。同时定义时必须初始化。
对于4
表示指针本身和指向的内容均为常量。
二、放在函数前后区别
1、int GetY() const;
2、const int * GetPosition();

对于1
该函数为只读函数，不允许修改其中的数据成员的值。

对于2
修饰的是返回值，表示返回的是指针所指向值是常量。
三、深入理解const
     const对象只能调用const成员函数；const对象的值不能被修改，在const成员函数中修改const对象数据成员的值是语法错误；在const函数中调用非const成员函数是语法错误。
 在一个类的函数后面加上const后，就表明这个函数是不能改变类的成员变量。如果在编写const成员函数时，不慎修改了数据成员，或者调用了其它非const成员函数，编译器将指出错误，这无疑会提高程序的健壮性。
以下程序中，类stack的成员函数GetCount仅用于计数，从逻辑上讲GetCount应当为const函数。编译器将指出GetCount函数中的错误。
class Stack
{
  public:
void  Push(int elem);
int  Pop(void);
int GetCount(void)  const; // const成员函数
  private:
int m_num;
int m_data[100];
};

int Stack::GetCount(void)  const
{
++ m_num; // 编译错误，企图修改数据成员m_num
Pop(); // 编译错误，企图调用非const函数
return m_num;
}
const成员函数的声明看起来怪怪的：const关键字只能放在函数声明的尾部，大概是因为其它地方都已经被占用了
*/