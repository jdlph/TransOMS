#include <handles.h>

#include <iostream>
#include <string>

using namespace opendta;
using namespace std::chrono;

int main()
{
    auto ts = high_resolution_clock::now();

    NetworkHandle nh;

    const std::string dir {"../data/"};
    nh.read_network(dir);
    nh.read_demands(dir);

    unsigned short column_gen_num = 20;
    unsigned short column_opt_num = 20;
    nh.find_ue(column_gen_num, column_opt_num);

    auto te = high_resolution_clock::now();
    std::cout << "openDTA finds UE in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";

    ts = high_resolution_clock::now();

    nh.output_columns(".");
    nh.output_link_performance(".");

    te = high_resolution_clock::now();
    std::cout << "openDTA outputs results in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
}