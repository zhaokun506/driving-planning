/*
  此类的功能主要用于构造参考线类型的变量
*/

#pragma once
#include "common/pnc_point.h"
#include <vector>

class ReferenceLine {

  // XYtoSL

  // SYtoXY

  //变量

  // x,y,
public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;
  void set_reference_points(std::vector<ReferencePoint> reference_points);
  void set_match_point_index(int index);
  void set_host_match_point(ReferencePoint host_match_point);
  void set_host_project_point(ReferencePoint host_project_point);

  const std::vector<ReferencePoint> reference_points() const;
  const int match_point_index() const;
  const ReferencePoint host_project_point() const;
  const ReferencePoint host_match_point() const;

private:
  std::vector<ReferencePoint> reference_points_; //参考点类
  int match_point_index_;                        //自车匹配点索引
  ReferencePoint host_project_point_;
  ReferencePoint host_match_point_;
};

/*C++中 public，protected， private 访问标号小结
第一：private, public, protected 访问标号的访问范围。

private：只能由1.该类中的函数、2.其友元函数访问。不能被任何其他访问，该类的对象也不能访问。
protected：可以被1.该类中的函数、2.子类的函数、以及3.其友元函数访问。但不能被该类的对象访问。
public：可以被1.该类中的函数、2.子类的函数、3.其友元函数访问，也可以由4.该类的对象访问。
注：友元函数包括3种：设为友元的普通的非成员函数；设为友元的其他类的成员函数；设为友元类中的所有成员函数。
第二：类的继承后方法属性变化。

private 属性不能够被继承。
使用private继承，父类的protected和public属性在子类中变为private；
使用protected继承，父类的protected和public属性在子类中变为protected；
使用public继承，父类中的protected和public属性不发生改变;如下所示：
public: protected: private:
public继承 public protected 不可用
protected继承 protected protected 不可用
private继承 private private 不可用
protected继承和private继承能降低访问权限。
*/