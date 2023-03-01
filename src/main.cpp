#include <handles.h>
#include <string>
#include <iostream>

using namespace opendta;

int main()
{   
    auto ts = std::chrono::high_resolution_clock::now();
    
    NetworkHandle nh;

    const std::string dir {"../data/"};
    nh.read_network(dir);
    nh.read_demands(dir);

    unsigned short column_gen_num = 20;
    unsigned short column_opt_num = 20;
    nh.find_ue(column_gen_num, column_opt_num);

    auto te = std::chrono::high_resolution_clock::now();
    std::cout << "openDTA finds UE in " << std::chrono::duration_cast<std::chrono::milliseconds>(te - ts).count() << " milliseconds\n";

}