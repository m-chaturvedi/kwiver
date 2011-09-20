/*ckwg +5
 * Copyright 2011 by Kitware, Inc. All Rights Reserved. Please refer to
 * KITWARE_LICENSE.TXT for licensing information, or contact General Counsel,
 * Kitware, Inc., 28 Corporate Drive, Clifton Park, NY 12065.
 */

#include <vistk/pipeline/process.h>

#include <vistk/config.h>

#include <boost/make_shared.hpp>

using namespace vistk;

class VISTK_NO_EXPORT test_process
  : public vistk::process
{
  public:
    test_process(config_t const& config);
    ~test_process();
};

test_process
::test_process(config_t const& config)
  : process(config)
{
}

test_process
::~test_process()
{
}

extern "C"
{

void VISTK_EXPORT register_processes();

}

static process_t create_test_process(config_t const& config);

void
register_processes()
{
  static process_registry::module_t const module_name = process_registry::module_t("test_processes");

  process_registry_t const registry = process_registry::self();

  if (registry->is_module_loaded(module_name))
  {
    return;
  }

  registry->register_process("test", "A test process", create_test_process);

  registry->mark_module_as_loaded(module_name);
}

process_t
create_test_process(config_t const& config)
{
  return boost::make_shared<test_process>(config);
}
