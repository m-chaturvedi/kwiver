/*ckwg +29
 * Copyright 2018 by Kitware, Inc.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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


#include "kwiver_applet.h"
#include "applet_context.h"

#include <vital/plugin_loader/plugin_manager.h>
#include <vital/plugin_loader/plugin_factory.h>

#include <kwiversys/SystemTools.hxx>

#include <cstdlib>
#include <iostream>

typedef kwiver::vital::implementation_factory_by_name< kwiver::tools::kwiver_applet > applet_factory;

//
// Global shared context
//
kwiver::tools::applet_context G_context;

// ============================================================================
int main(int argc, char *argv[])
{
  kwiver::vital::plugin_manager& vpm = kwiver::vital::plugin_manager::instance();
  vpm.load_all_plugins();

  // initialize the global context
  G_context.m_ostream = &std::cout;
  G_context.m_wtb.set_indent_string( "      " );

  // Test to see if there are any args
  if ( (argc < 2) || (std::string(argv[1]) == "help") )
  {
    // display help message
    std::cout << "Usage: " << argv[0] << " <applet?  [args]" << std::endl
              << "<applet> can be one of the following:" << std::endl
              << "help - prints this message" << std::endl
      ;

    // Get list of factories for implementations of the applet
    const auto fact_list = vpm.get_factories( typeid( kwiver::tools::kwiver_applet ).name() );

    // Loop over all factories in the list and display name and description
    for( auto fact : fact_list )
    {
      std::string buf = "-- Not Set --";
      fact->get_attribute( kwiver::vital::plugin_factory::PLUGIN_NAME, buf );
      std::cout << "    " << buf << " - ";

      fact->get_attribute( kwiver::vital::plugin_factory::PLUGIN_DESCRIPTION, buf );
      std::cout << G_context.m_wtb.wrap_text( buf );
    }

    exit(0);
  } // end help code

  // ----------------------------------------------------------------------------
  try
  {
    // Create applet based on the name provided
    applet_factory app_fact;
    auto applet = app_fact.create( argv[1] );
    applet->initialize( &G_context );

    // Edit the command line args
    // TBD Need to add support for -o <file> flag to direct output to a file.

    // TBD Add support for converting <applet-name> "help" into <applet-name> "--help"
    // This requires that all applets support --help

    // create local args for tool
    int local_argc(argc-1);
    char** local_argv;
    local_argv = static_cast< char** >(calloc( sizeof(char*), argc-1 ));

    local_argv[0] = argv[0];
    for (int i = 1; i < argc -1; ++i )
    {
      local_argv[i] = argv[i+1];
    }

    // Run the specified tool
    return applet->run( local_argc, local_argv );
  }
  catch ( kwiver::vital::plugin_factory_not_found& )
  {
    std::cerr << "Tool \"" << argv[1] << "\" not found. Type \""
              << argv[0] << " help\" to list available tools." << std::endl;

    exit(-1);
  }

  return 0;
}
