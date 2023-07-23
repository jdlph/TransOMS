/**
 * @file main.cpp, part of the project TransOMS under Apache License 2.0
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Entry point to TransOMS
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 */

#include <handles.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace transoms;
using namespace std::chrono;

int main()
{
    auto ts = high_resolution_clock::now();

    // const std::string dir {"../data/Chicago_Regional/"};
    const std::string dir {"../data/Chicago_Sketch/"};
    // const std::string dir {"../data/Two_Corridor/"};
    // const std::string dir {"../data/Sioux_Falls/"};

    NetworkHandle nh;
    nh.read_settings(dir);
    nh.read_network(dir);
    nh.read_demands(dir);

    auto te = high_resolution_clock::now();
    std::cout << "TransOMS loads input in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
    ts = high_resolution_clock::now();

    unsigned short column_gen_num = 20;
    unsigned short column_opt_num = 20;
    nh.find_ue(column_gen_num, column_opt_num);

    te = high_resolution_clock::now();
    std::cout << "TransOMS finds UE in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
    ts = high_resolution_clock::now();

    nh.run_simulation();

    te = high_resolution_clock::now();
    std::cout << "TransOMS completes DTA in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
    ts = high_resolution_clock::now();

    // nh.output_columns();
    // nh.output_link_performance();
    // nh.output_trajectories();

    te = high_resolution_clock::now();
    std::cout << "TransOMS outputs results in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
}