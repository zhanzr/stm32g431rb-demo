#include <iostream>
#include <memory>
#include <type_traits>

using namespace std;

struct X {
    X() = default;
};

struct Y {
    Y() { };
};

class A {
public:
    int x;
    A(){}
};

class B {
public:
    int x;
    B()=default;

		virtual void func_final(void) final {
			cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
		}
		
		virtual void func_override(void){
			cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
		}
};

class B_child : public B {
public:
    int x;
//    B_child()=delete;
    B_child()=default;
    ~B_child()=default;

//		virtual void func_final(void) override{
//			cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
//		}
		virtual void func_override(void) override{
			cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
		}
};

void test_default_1(void) {
    static_assert(std::is_trivial<X>::value, "X should be trivial");
    static_assert(std::is_pod<X>::value, "X should be POD");
    
    static_assert(!std::is_trivial<Y>::value, "Y should not be trivial");
    static_assert(!std::is_pod<Y>::value, "Y should not be POD");
}

void test_default_2(void) {
    int x = 5;
    auto a = A();
    cout << a.x << endl;
	
    B b;
    cout << b.x << endl;
	
    auto b_1 = B();
    cout << b_1.x << endl;
		b_1.func_final();
		b_1.func_override();

    auto b_child = B_child();
    cout << b_child.x << endl;
		b_child.func_final();
		b_child.func_override();
}
