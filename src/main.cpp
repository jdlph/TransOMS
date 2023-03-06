/**
 * @file main.cpp, part of the project openDTA under GPL-3.0 license
 * @author Peiheng Li (jdlph@hotmail.com)
 * @brief Entry point to openDTA
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 *
 */

#include <handles.h>

#include <iostream>
#include <string>
#include <thread>
#include <functional>

using namespace opendta;
using namespace std::chrono;

int main()
{
    std::ios_base::sync_with_stdio(false);

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

    // multithreading to handle I/O-bounded processes (not functioning yet!)
    // std::thread t1 (&NetworkHandle::output_columns, nh);
    // std::thread t2 (&NetworkHandle::output_link_performance, nh);

    // t1.join();
    // t2.join();

    nh.output_columns();
    nh.output_link_performance();

    te = high_resolution_clock::now();
    std::cout << "openDTA outputs results in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
}