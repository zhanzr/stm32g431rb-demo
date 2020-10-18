#include <iostream>
#include <memory>
using namespace std;

struct A {
  double x;
};
unique_ptr<A> a;

decltype(a->x) y;       // type of y is double (declared type)
decltype((a->x)) z = y; // type of z is const double& (lvalue expression)

template <typename T, typename U>
auto add(T t, U u)
    -> decltype(t + u) // return type depends on template parameters
                       // return type can be deduced since C++14
{
  return t + u;
}

void test_decltype(void) {
  int i = 33;
  decltype(i) j = i * 2;

  cout << "a:" << reinterpret_cast<void*>(a.get()) << endl;
	a.reset(new A);
  cout << "a:" << reinterpret_cast<void*>(a.get()) << endl;

  cout << "i = " << i << ", "
       << "j = " << j << endl;

  auto f = [](int a, int b) -> int { return a * b; };

  decltype(f) g = f; // the type of a lambda function is unique and unnamed
  i = f(2, 2);
  j = g(3, 3);
  cout << "f:" << f << endl;
  cout << "g:" << g << endl;

  cout << "i = " << i << ", "
       << "j = " << j << endl;
}