#include <gtest/gtest.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int i, int j)
{
    return i + j;
}

int sub(int i, int j)
{
    return i - j;
}

PYBIND11_MODULE(example, m)
{
    m.doc() = "pybind11 example plugin";
    m.def("add", &add, "A function that adds two numbers.", py::arg("i") = 1, py::arg("j") = 2);
    auto m_math_utils = m.def_submodule("math_utils", "last_letter_lib math_utils submodule");
    m_math_utils.def("sub", &sub, "A function that subs two numbers.", py::arg("i") = 2, py::arg("j") = 1);
}

// TEST(PybindTest, Test1)
// {
// }
