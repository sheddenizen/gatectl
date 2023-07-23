#ifndef TUNABLE_HPP
#define TUNABLE_HPP

#include "cli.hpp"

#include <string>

namespace tunable {


template<typename T>
class Item
{
  public:
    Item(std::string const & name, T default_value, cli::Executor &cli_exec)
      : _default(default_value)
      , _value(default_value)
    {
      our_cli_exec() = &cli_exec;
      register_cmds(name, cli_exec);
    }
    Item(std::string const & name, T default_value)
      : _default(default_value)
      , _value(default_value)
    {
      if (our_cli_exec()) {
        register_cmds(name, *our_cli_exec());
      }
    }
    void register_cmds(std::string const & name, cli::Executor & cli_exec)
    {
      cli_exec.add_command(std::string("set-") + name, [this](T value){return this->set(value);}, std::string("Set ") + name + " tunable to value" );
      cli_exec.add_command(std::string("get-") + name, [this](){return (*this)();}, std::string("Get ") + name + " tunable current value" );
      cli_exec.add_command(std::string("def-") + name, [this](){return this->set_default();}, std::string("Restore ") + name + " tunable to default value (" + std::to_string(_default) + ")" );
    }
    operator T() const { return _value; }
    T operator() () const { return _value; }
    T set(T value) { _value = value; return _value; }
    T set_default() { _value = _default; return _value; }    
  private:
    T const _default;
    T _value;
    Item():_default(0){}
    cli::Executor * & our_cli_exec()
    {
      // Nasty, but convenient
      static cli::Executor * cli_exec;
      return cli_exec;
    }
    friend void set_cli_executor(cli::Executor & cli_exec);
};

inline void set_cli_executor(cli::Executor & cli_exec)
{
  Item<int>().our_cli_exec() = &cli_exec;
}

} // tunable

#endif // TUNABLE_HPP