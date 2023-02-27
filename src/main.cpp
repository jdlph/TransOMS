#include <handles.h>
#include <string>

using namespace opendta;

int main()
{   
    NetworkHandle nh;

    const std::string dir {"../data/"};
    nh.read_network(dir);
    nh.read_demands(dir);

    unsigned short column_gen_num = 20;
    unsigned short column_opt_num = 20;
    nh.find_ue(column_gen_num, column_opt_num);
}