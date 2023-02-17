
#include <iostream>
#include <thread> 
#include <fstream>
#include <chrono>
void *childThread(void *arg); 
using namespace std;
 
class Resource {
public:
    Resource(int n) {
        this->n_ = n;       
        std::cout << " 3" <<  std::endl;

        std::cout << "Create Resource." << endl;
    }
    virtual ~Resource() {

        std::cout << "free Resource." << endl;
    }
private:
    int n_;
 
};
 void run()
 {
    std::thread t1 = std::thread(childThread, nullptr);
    std::cout << " 11" <<  std::endl;
    using ms = chrono::milliseconds;
    std::this_thread::sleep_for(ms(20));
    std::cout << " wake up" <<  std::endl;
    // while (true) {
    //    std::this_thread::sleep_for(ms(50));
    //    std::cout << "8" <<  std::endl;
    // }
    // t1.join();
    t1.detach();

 }
int main(int argc, char **argv) {
    run();
    using ms = chrono::milliseconds;
    std::cout << "main function" << std::endl;
    // while (true) {
    //    std::this_thread::sleep_for(ms(500));
    //    std::cout << "5" <<  std::endl;
    // }
    return 0;
}

void *childThread(void *arg) {
    Resource r(5);
    std::cout << " 2" <<  std::endl;
    using ms = chrono::milliseconds;
    while (true) {
        // std::cout << "7" <<  std::endl;
       std::this_thread::sleep_for(ms(20));
       std::cout << "4"<<  std::endl ;
       std::this_thread::sleep_for(ms(20));
    }
    return nullptr;
 
}
