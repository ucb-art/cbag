// SPDX-License-Identifier: BSD-3-Clause
/*
Copyright (c) 2018, Regents of the University of California
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <map>
#include <utility>
#include <variant>

#include <fmt/core.h>

#include <cbag/spirit/ast.h>

#include <cbag/schematic/cellview.h>
#include <cbag/schematic/cellview_info.h>
#include <cbag/schematic/instance.h>

#include <cbag/netlist/lstream.h>
#include <cbag/netlist/ngspice.h>
#include <cbag/spirit/util.h>
#include <cbag/util/io.h>
#include <cbag/util/name_convert.h>


namespace cbag {
namespace netlist {

// ngspice netlisting helper functions
namespace ngspice {

using term_net_vec_t = std::vector<std::pair<cnt_t, std::vector<std::string>>>;

void get_cv_pins(lstream &b, const std::vector<std::string> &names) {
    for (auto const &name : names) {
        spirit::ast::name_unit ast = cbag::util::parse_cdba_name_unit(name);
        auto n = ast.size();
        for (decltype(n) idx = 0; idx < n; ++idx) {
            std::string tmp = ast[idx].to_string(false, spirit::namespace_ngspice{});
            b << tmp;
        }
    }
}

std::unordered_map<std::string, std::string> new_cell_name_map() {
    // map CDF cell names to ngspice cell names
    // TODO: switch. requires a model.
    auto cell_map = std::unordered_map<std::string, std::string>();
    cell_map["cap"] = "";
    cell_map["dcblock"] = "";
    cell_map["idc"] = "dc";
    cell_map["ind"] = "";
    cell_map["dcfeed"] = "";
    cell_map["ipulse"] = "pulse";
    cell_map["ipwlf"] = "pwl";
    cell_map["isin"] = "sin";
    cell_map["res"] = "";
    cell_map["vdc"] = "dc";
    cell_map["vpulse"] = "pulse";
    cell_map["vpwlf"] = "pwl";
    cell_map["vsin"] = "sin";
    // Ngspice specific
    cell_map["vtrnoise"] = "trnoise";
    cell_map["itrnoise"] = "trnoise";
    return cell_map;
}

std::unordered_map<std::string, std::vector<std::string>> primitive_map() {
    // Map ckt primitives to the key order
    // Ngspice does not support assign by name, only by order
    auto prim_map = std::unordered_map<std::string, std::vector<std::string>>();
    // Passives
    prim_map["cap"] = {"c"};
    prim_map["dcblock"] = {"c"};
    prim_map["res"] = {"r"};
    prim_map["ind"] = {"l"};
    prim_map["dcfeed"] = {"l"};
    //Sources, single value
    //  -> Handled directly below
    //Sources, multivalue
    prim_map["pwl"] = {"value"};
    prim_map["ac"] = {"acm", "acp"};
    prim_map["pulse"] = {"v1", "v2", "td", "tr", "tf", "pw", "per"};
    prim_map["sin"] = {"vo", "va", "freq", "td", "theta", "phase"};
    prim_map["trnoise"] = {"na", "nt", "nalpha", "namp"};
    return prim_map;
}

std::unordered_map<std::string, std::unordered_map<std::string, std::string>> new_cell_prop_map() {
    // port specific parameters
    auto cp_map = std::unordered_map<std::string, std::unordered_map<std::string, std::string>>();
    auto prop_map = std::unordered_map<std::string, std::string>();
    prop_map["filenums"] = "";
    prop_map["pm"] = "";
    cp_map.emplace("port", std::move(prop_map));
    return cp_map;
}

template <class OutIter> class write_param_alt {
  // Class for writing most primitives for Ngspice
  private:
    OutIter &iter_;
    std::string dbl_fmt_;

  public:
    write_param_alt(OutIter &iter, uint_fast32_t precision)
        : iter_(iter), dbl_fmt_(fmt::format("{{}}={{:.{}g}}", precision)) {}

    void operator()(const std::string &v) const {
        // TODO: sanity checking the keys, perhaps with v.empty().
        //   Better is to include the key name.
        *iter_ = fmt::format("'{}'", v);
    }
    void operator()(const int_fast32_t &v) const { *iter_ = fmt::format("{}", v); }
    void operator()(const double_t &v) const { *iter_ = fmt::format(dbl_fmt_, v); }
    void operator()(const bool &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("bool parameter, do nothing.");
    }
    void operator()(const time_struct &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("time parameter, do nothing.");
    }
    void operator()(const binary_t &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("binary parameter, do nothing.");
    }
};

template <class OutIter> class write_raw {
  // Class for writing raw strings.
  private:
    OutIter &iter_;

  public:
    write_raw(OutIter &iter): iter_(iter) {}

    void operator()(const std::string &v) const {*iter_ = v;}
    void operator()(const int_fast32_t &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("int parameter, do nothing.");
    }
    void operator()(const double_t &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("double parameter, do nothing.");
    }
    void operator()(const bool &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("bool parameter, do nothing.");
    }
    void operator()(const time_struct &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("time parameter, do nothing.");
    }
    void operator()(const binary_t &v) const {
        auto logger = cbag::get_cbag_logger();
        logger->warn("binary parameter, do nothing.");
    }
};

template <class OutIter>
void write_instance_cell_name(OutIter &&iter, const param_map &params,
                              const sch::cellview_info &info, ngspice_stream &stream) {
    bool is_prim = (info.lib_name == "analogLib" || info.lib_name == "basic" 
        || info.lib_name == "ngspice");

    auto &cur_cell_name = is_prim
                              ? stream.get_cell_name(info.cell_name)
                              : info.cell_name;

    // get default parameter values
    param_map par_map(info.props);
    // update with instance parameters
    for (auto const & [ key, val ] : params) {
        par_map.insert_or_assign(key, val);
    }

    // Early escape
    if (par_map.size() < 1) {
        *iter = cur_cell_name;
        return;
    }

    // write instance parameters
    cnt_t precision = stream.precision();
    if (is_prim) {
        // Ngspice does not support assign by name, only by order
        auto prim_map = primitive_map();
        std::vector<std::string> param_list;

        if (info.cell_name == "port") {
            // Sanity check for values
            for (const std::string &key : {"vdc", "acm", "num"}) {
                auto val_loc = par_map.find(key);
                if (val_loc == par_map.end())
                    throw std::runtime_error("Ngspice requires key for RF port: " + key);
            }

            // Just do this manually
            // dc <val> ac <val> portnum <num>
            std::visit(write_raw(iter), value_t(std::string("dc")));
            std::visit(write_param_alt(iter, precision), par_map.find("vdc")->second);
            std::visit(write_raw(iter), value_t(std::string("ac")));
            std::visit(write_param_alt(iter, precision), par_map.find("acm")->second);
            std::visit(write_raw(iter), value_t(std::string("portnum")));
            std::visit(write_param_alt(iter, precision), par_map.find("num")->second);
        }
        else if ((info.cell_name == "vdc" || info.cell_name == "idc")) {
            // DC and AC sources in Ngspice are combined and may have both a DC and an AC component.
            std::visit(write_raw(iter), value_t(std::string("dc")));
            std::visit(write_param_alt(iter, precision), par_map.find("vdc")->second);

            if (par_map.find("acm") != par_map.end()) {
                if (std::holds_alternative<std::string>(par_map.find("acm")->second)) {
                    if ((std::get<std::string>(par_map.find("acm")->second)).empty()) {
                        return;
                    }
                }
                std::visit(write_raw(iter), value_t(std::string("ac")));
                std::visit(write_param_alt(iter, precision), par_map.find("acm")->second);
            }
            return;
        }
        else if ((info.cell_name == "vsin" || info.cell_name == "isin") & (par_map.find("acm") != par_map.end())) {
            // Two ways to use vsin -> AC source or tran source.
            // This is the escape case for AC source.
            if (par_map.find("vdc") != par_map.end()) {
                if (std::holds_alternative<std::string>(par_map.find("vdc")->second)) {
                    if (!(std::get<std::string>(par_map.find("vdc")->second)).empty()) {
                        std::visit(write_raw(iter), value_t(std::string("dc")));
                        std::visit(write_param_alt(iter, precision), par_map.find("vdc")->second);
                    }
                } else {
                    std::visit(write_raw(iter), value_t(std::string("dc")));
                    std::visit(write_param_alt(iter, precision), par_map.find("vdc")->second);
                }
            }

            std::visit(write_raw(iter), value_t(std::string("ac")));
            std::visit(write_param_alt(iter, precision), par_map.find("acm")->second);
            return;
        }
        else if ((info.cell_name == "vpwlf" || info.cell_name == "ipwlf")) {
            // Ngspice expects a time-value pairs. We will pass in a raw string with these values.
            std::visit(write_raw(iter), value_t(std::string("pwl")));
            std::visit(write_raw(iter), par_map.find("value")->second);
            return;
        }
        else {
            // Search first by original name, then by new name.
            auto val_loc = prim_map.find(info.cell_name);
            if (val_loc == prim_map.end()) {
                val_loc = prim_map.find(cur_cell_name);
                if (val_loc == prim_map.end()) 
                    throw std::runtime_error("Unsupported or not implemented basic type for Ngspice plugin: " + info.cell_name + " / " + cur_cell_name);
            }
            param_list = val_loc->second;
            *iter = cur_cell_name;
        }
        
        for (const std::string &key : param_list) {
            auto val_loc = par_map.find(key);
            if (val_loc == par_map.end())
                throw std::runtime_error("Source or prim parameter not found for cell " + info.cell_name + ": " + key);
            std::visit(write_param_alt(iter, precision), val_loc->second);
        }
    } else {
        *iter = cur_cell_name;
        for (auto const & [ key, val ] : par_map) {
            std::visit(write_param_visitor(iter, key, precision), val);
        }
    }
}

} // namespace ngspice

ngspice_stream::ngspice_stream()
    : nstream_output(), cell_name_map_(ngspice::new_cell_name_map()),
      cell_prop_map_(ngspice::new_cell_prop_map()) {}

ngspice_stream::ngspice_stream(const std::string &fname, cnt_t precision)
    : nstream_output(fname), precision_(precision), cell_name_map_(ngspice::new_cell_name_map()),
      cell_prop_map_(ngspice::new_cell_prop_map()) {}

const std::string &ngspice_stream::get_cell_name(const std::string &cell_name) const {
    auto iter = cell_name_map_.find(cell_name);
    return (iter == cell_name_map_.end()) ? cell_name : iter->second;
}
cnt_t ngspice_stream::precision() const noexcept { return precision_; }

void traits::nstream<ngspice_stream>::close(type &stream) { stream.close(); }

void traits::nstream<ngspice_stream>::write_header(type &stream,
                                                   const std::vector<std::string> &inc_list,
                                                   bool shell) {
    // set language
    stream << "*NGSPICE netlist\n";

    if (!shell) {
        if (!inc_list.empty()) {
            for (auto const &fname : inc_list) {
                stream << ".include \"" << util::get_canonical_path(fname).c_str() << "\"\n";
            }
            stream << '\n';
        }
    }
}

void traits::nstream<ngspice_stream>::write_end(type &stream) {}

void traits::nstream<ngspice_stream>::write_cv_header(type &stream, const std::string &name,
                                                      const sch::cellview_info &info, bool shell,
                                                      bool write_subckt, bool write_declarations) {
    stream << "\n\n";
    if (write_subckt) {
        lstream b;
        b << ".subckt";
        b << name;
        ngspice::get_cv_pins(b, info.out_terms);
        ngspice::get_cv_pins(b, info.io_terms);
        ngspice::get_cv_pins(b, info.in_terms);

        // write definition line
        b.to_file(stream, spirit::namespace_ngspice{});
    }
}

void traits::nstream<ngspice_stream>::write_cv_end(type &stream, const std::string &name,
                                                   bool write_subckt) {
    if (write_subckt) {
        stream << ".ends " << name << '\n';
    } else {
        stream << '\n';
    }
}

void traits::nstream<ngspice_stream>::write_unit_instance(
    type &stream, const std::string &prefix, cnt_t inst_idx, const spirit::ast::name_unit &name_ast,
    const term_net_vec_t &conn_list, const param_map &params, const sch::cellview_info &info,
    const net_rename_map_t *net_map_ptr) {

    auto tag = spirit::namespace_ngspice{};
    auto name = name_ast[inst_idx].to_string(true, tag);

    auto b = lstream();
    // write instance name
    b << (prefix + name);

    // write instance nets
    for (const auto & [ term_ast, net_ast_list ] : conn_list) {
        spirit::util::get_name_bits(net_ast_list[inst_idx],
                                    b.get_bit_inserter(prefix, net_map_ptr, tag));
    }

    // write instance cell name and properties
    ngspice::write_instance_cell_name(b.get_back_inserter(), params, info, stream);
    b.to_file(stream, spirit::namespace_cdba{});
}

void traits::nstream<ngspice_stream>::append_netlist(type &stream, const std::string &netlist) {
    stream << "\n\n";
    stream << netlist;
}

void traits::nstream<ngspice_stream>::write_supply_wrapper(type &stream, const std::string &name,
                                                           const sch::cellview_info &info) {
    throw std::runtime_error("supply wrapping not supporting in ngspice netlist.");
}

net_rename_map_t traits::nstream<ngspice_stream>::new_net_rename_map() {
    auto ans = net_rename_map_t();
    ans.emplace(spirit::ast::name_bit("gnd!"), spirit::ast::name_bit("0"));
    return ans;
}

} // namespace netlist
} // namespace cbag
