/*ckwg +29
 * Copyright 2017 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pybind11/stl.h>

#include "descriptor_class.cxx"

namespace py = pybind11;

class PyDescriptorSet
{
  std::vector<std::shared_ptr<PyDescriptorBase>> descriptors;

  public:

    PyDescriptorSet() {};
    PyDescriptorSet(py::list desc_arg)
     {
       for(auto py_desc: desc_arg)
       {
         std::shared_ptr<PyDescriptorBase> desc = py_desc.cast<std::shared_ptr<PyDescriptorBase>>();
         descriptors.push_back(desc);
       }
     };

    size_t size() { return descriptors.size(); };

    std::vector<std::shared_ptr<PyDescriptorBase>> get_descriptors() { return descriptors; };
};

PYBIND11_MODULE(_descriptor_set, m)
{
  py::class_<PyDescriptorSet, std::shared_ptr<PyDescriptorSet>>(m, "DescriptorSet")
  .def(py::init<>())
  .def(py::init<py::list>())
  .def("descriptors", &PyDescriptorSet::get_descriptors)
  .def("size", &PyDescriptorSet::size)
  .def("__len__", &PyDescriptorSet::size)
  ;
}
