// Copyright (c) 2022, Grzegorz Bartyzel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <string>
#include <vector>

#include "net_ft_driver/interfaces/ati_ft_interface.hpp"

constexpr uint32_t kBias = 0x0042;

namespace net_ft_driver
{
AtiFTInterface::AtiFTInterface(const std::string& ip_address) : NetFTInterface(ip_address, 7000)
{
}

bool AtiFTInterface::set_cgi_variable(const std::string& cgi_name, const std::string& var_name,
                                      const std::string& value)
{
  try {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;
    std::string xml_url{ "http://" + ip_address_ + "/" + cgi_name + "?" + var_name + "=" + value };
    request.setOpt(new curlpp::options::Url(xml_url));
    request.perform();
    return true;
  } catch (curlpp::RuntimeError& e) {
    std::cerr << "[set_cgi_variable] " << e.what() << std::endl;
  } catch (curlpp::LogicError& e) {
    std::cerr << "[set_cgi_variable] " << e.what() << std::endl;
  }
  return false;
}

bool AtiFTInterface::set_cgi_variables(const std::string& cgi_name, const std::vector<std::string>& var_names,
                                      const std::vector<std::string>& values)
{
  if (var_names.size() != values.size() ) {
    std::cerr << "[set_cgi_variable] var_names and values have different sizes" << std::endl;
    return false;
  }
  try {
    curlpp::Cleanup cleanup;
    curlpp::Easy request;
    std::string xml_url{ "http://" + ip_address_ + "/" + cgi_name + "?"};
    for (size_t i=0; i<var_names.size()-1; i++) {
      xml_url.append(var_names.at(i) + "=" + values.at(i) + "&");
    }
    xml_url.append(var_names.back() + "=" + values.back());

    request.setOpt(new curlpp::options::Url(xml_url));
    request.perform();
    return true;
  } catch (curlpp::RuntimeError& e) {
    std::cerr << "[set_cgi_variable] " << e.what() << std::endl;
  } catch (curlpp::LogicError& e) {
    std::cerr << "[set_cgi_variable] " << e.what() << std::endl;
  }
  return false;
}

bool AtiFTInterface::set_bias()
{
  return send_command(kBias);
}

bool AtiFTInterface::set_bias(const std::array<double,6>& biases)
{
  auto ret = set_cgi_variables("setting.cgi", 
    {"setbias0", "setbias1", "setbias2", "setbias3", "setbias4", "setbias5"},
    {std::to_string(biases.at(0)), std::to_string(biases.at(1)), std::to_string(biases.at(2)),
      std::to_string(biases.at(3)), std::to_string(biases.at(4)), std::to_string(biases.at(5))}
  );
  return ret;
}

bool AtiFTInterface::clear_bias()
{
  return set_bias({0,0,0,0,0,0});
}

/**
Sets the RDT output rate in Hertz. The actual value used
may be rounded up; see Section 4.7—Communication
Settings Page (comm.htm) for details.
*/
bool AtiFTInterface::set_sampling_rate(int rate)
{
  if (rate < 1 || rate > 7000) {
    std::cerr << "Sampling rate our of value, allowed range: 1-7000";
    return false;
  }
  return set_cgi_variable("comm.cgi", "comrdtrate", std::to_string(rate));
}

/**
Filter Index | Cutoff
-------------|-----------
0            | no filter
1            | 838 Hz
2            | 326 Hz
3            | 152 Hz
4            | 73 Hz
5            | 35 Hz
6            | 18 Hz
7            | 8 Hz
8            | 5 Hz
9            | 1500 Hz
10           | 2000 Hz
11           | 2500 Hz
12           | 3000 Hz
*/
bool AtiFTInterface::set_internal_filter(int value)
{
  if (value < 0 || value > 12) {
    std::cerr << "Filter value out of the range, allowed range: 0-12";
    return false;
  }
  return set_cgi_variable("setting.cgi", "setuserfilter", std::to_string(value));
}
}  // namespace net_ft_driver
