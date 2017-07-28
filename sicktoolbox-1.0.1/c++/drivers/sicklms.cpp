#include <boost/python.hpp>
#include <sicklms-1.0/SickLMS.hh>
using namespace boost::python;
using namespace SickToolbox;

BOOST_PYTHON_MODULE(sicklms)
{
    class_<SickLMS>("SickLMS")
        .def("__init__", &SickLMS::greet)
        .def("initialize", &SickLMS::Initialize)
    ;
}
