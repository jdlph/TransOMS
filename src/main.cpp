/**
 * @file main.cpp, part of the project openDTA under GPL-3.0 license
 * @author jdlph (jdlph@hotmail.com) and xzhou99 (xzhou74@asu.edu)
 * @brief Entry point to openDTA
 *
 * @copyright Copyright (c) 2023 Peiheng Li, Ph.D. and Xuesong (Simon) Zhou, Ph.D.
 *
 */

#include <handles.h>

#include <iostream>
#include <string>

using namespace opendta;
using namespace std::chrono;

int main()
{
    auto ts = high_resolution_clock::now();

    std::ios_base::sync_with_stdio(false);

    const std::string dir {"../data/Chicago_Regional/"};
    // const std::string dir {"../data/Chicago_Sketch/"};

    NetworkHandle nh;
    nh.read_network(dir);
    nh.read_demands(dir);

    unsigned short column_gen_num = 20;
    unsigned short column_opt_num = 20;
    nh.find_ue(column_gen_num, column_opt_num);

    auto te = high_resolution_clock::now();
    std::cout << "openDTA finds UE in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";

    ts = high_resolution_clock::now();

    nh.output_columns_par();
    nh.output_link_performance();

    te = high_resolution_clock::now();
    std::cout << "openDTA outputs results in " << duration_cast<milliseconds>(te - ts).count() << " milliseconds\n";
}