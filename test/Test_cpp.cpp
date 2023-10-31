#include <memory>
#include <iostream>
#include <unordered_map>

class A {
public:
    typedef std::shared_ptr<A> Ptr;

    static A::Ptr createA() { return A::Ptr(new A()); }

private:
    double value;
};

class B {
public:
    B() {}

    ~B() {}

    void addElement(long i, A::Ptr frame) {
        frames[i] = frame;
    }

private:
    std::string name = "b";
    std::unordered_map<long, A::Ptr> frames;
};

int main(int argc, char **argv) {
    A::Ptr a = A::createA();
    B b;
    b.addElement(1, a);
    std::cout << "success" << std::endl;
    return 0;
}
