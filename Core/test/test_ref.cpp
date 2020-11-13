#include <iostream>
#include <string>
#include <vector>
#include <cassert>

using namespace std;

char& char_number(std::string& s, std::size_t n) {
    return s.at(n); // string::at() returns a reference to char
}
 
auto test_ref0(void) -> void {
    std::string str = __func__;
    char_number(str, 1) = 'a'; // the function call is lvalue, can be assigned to
    std::cout << str << '\n';
}

auto test_ref1(void) -> void {
    std::string s1 = __func__;
//  std::string&& r1 = s1;           // error: can't bind to lvalue
 
    const std::string& r2 = s1 + s1; // okay: lvalue reference to const extends lifetime
//  r2 += "Test";                    // error: can't modify through reference to const
 
    std::string&& r3 = s1 + s1;      // okay: rvalue reference extends lifetime
    r3 += __FILE__;                    // okay: can modify through reference to non-const
    std::cout << r3 << '\n';
}

void f(int& x) {
	cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;	
    std::cout << "lvalue reference overload f(" << x << ")\n";
}
 
void f(const int& x) {
	cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
	
    std::cout << "lvalue reference to const overload f(" << x << ")\n";
}
 
void f(int&& x) {
		cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
    std::cout << "rvalue reference overload f(" << x << ")\n";
}
 
auto test_ref2(void) -> void {
    int i = 1;
    constexpr const int ci = 2;
    f(i);  // calls f(int&)
    f(ci); // calls f(const int&)
    f(3);  // calls f(int&&)
           // would call f(const int&) if f(int&&) overload wasn't provided
    f(std::move(i)); // calls f(int&&)
 
    // rvalue reference variables are lvalues when used in expressions
    int&& x = 1;
    f(x);            // calls f(int& x)
    f(std::move(x)); // calls f(int&& x)
}

auto test_ref3(void) -> void {
		cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
		int i2 = 42;
		int&& rri = std::move(i2); // binds directly to i2
		cout << rri << endl;
		i2 = 123;
		cout << rri << endl;

		cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
		std::vector<int> v{1,2,3,4,5};
		std::vector<int> v2(std::move(v)); // binds an rvalue reference to v
		
		cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;
		for(auto vIt: v) {
			cout << vIt << endl;
		}
		cout << v.size() << ' ' << v.capacity() << endl;
		cout << v2.size() << ' ' << v2.capacity() << endl;

		cout << ' ' << __FILE__ << ' ' << __func__ << ' ' << __LINE__ << endl;		
		for(auto vIt: v2) {
			cout << vIt << endl;
		}
		cout << v.size() << ' ' << v.capacity() << endl;
		cout << v2.size() << ' ' << v2.capacity() << endl;
		
		assert(v.empty());	
}