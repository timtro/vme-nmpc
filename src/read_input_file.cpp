/*
 * read_input_file.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-20
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */


//[debug_settings_includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>
namespace pt = boost::property_tree;
//]
//[debug_settings_data
struct debug_settings {
  std::string m_file;               // log filename
  int m_level;                      // debug level
  std::set<std::string> m_modules;  // modules where logging is enabled
  void load(const std::string &filename);
  void save(const std::string &filename);
};
//]
//[debug_settings_load
void debug_settings::load(const std::string &filename) {
  // Create empty property tree object
  pt::ptree tree;

  // Parse the XML into the property tree.
  pt::read_xml(filename, tree);

  // Use the throwing version of get to find the debug filename.
  // If the path cannot be resolved, an exception is thrown.
  m_file = tree.get<std::string>("debug.filename");

  // Use the default-value version of get to find the debug level.
  // Note that the default value is used to deduce the target type.
  m_level = tree.get("debug.level", 0);

  // Use get_child to find the node containing the modules, and iterate over
  // its children. If the path cannot be resolved, get_child throws.
  // A C++11 for-range loop would also work.
  BOOST_FOREACH(pt::ptree::value_type &v, tree.get_child("debug.modules")) {
    // The data function is used to access the data stored in a node.
    m_modules.insert(v.second.data());
  }

}
//]
//[debug_settings_save
void debug_settings::save(const std::string &filename) {
  // Create an empty property tree object.
  pt::ptree tree;

  // Put the simple values into the tree. The integer is automatically
  // converted to a string. Note that the "debug" node is automatically
  // created if it doesn't exist.
  tree.put("debug.filename", m_file);
  tree.put("debug.level", m_level);

  // Add all the modules. Unlike put, which overwrites existing nodes, add
  // adds a new node at the lowest level, so the "modules" node will have
  // multiple "module" children.
  BOOST_FOREACH(const std::string &name, m_modules)
  tree.add("debug.modules.module", name);

  // Write property tree to XML file
  pt::write_xml(filename, tree);
}
//]

int main() {
  try {
    debug_settings ds;
    ds.load("debug_settings_xml");
    ds.save("debug_settings_out.xml");
    std::cout << "Success\n";
  } catch(std::exception &e) {
    std::cout << "Error: " << e.what() << "\n";
  }
  return 0;
}