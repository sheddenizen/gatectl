#include <functional>
#ifndef TUNABLE_HPP
#define TUNABLE_HPP

#include "cli.hpp"

#include <string>

namespace tunable {


template<typename T>
class Item
{
  public:
    Item(std::string name, T default_value)
      : _default(default_value)
      , _value(default_value)
    {
      // I have backed myself into a corner
      char * n = (char *)malloc(name.size() + 1);
      strcpy(n, name.c_str());
      _name = n;
      deferred_register();
    }
    Item(char const * name, T default_value)
      : _default(default_value)
      , _value(default_value)
      , _name(name)
    {
      deferred_register();
    }
    struct ILType { char const * name; T default_value; };
    Item(std::initializer_list<ILType>il)
      : _default(il.default_value)
      , _value(il.default_value)
      , _name(il.name)
    {
      deferred_register();
    }
    void register_cmds(cli::Executor & cli_exec)
    {
      std::string name(_name);
      cli_exec.add_command(std::string("set-") + name, [this](T value){return this->set(value);}, std::string("Set ") + name + " tunable to value (default value: " + std::to_string(_default) + ")" );
      cli_exec.add_command(std::string("get-") + name, [this](){return (*this)();}, std::string("Get ") + name + " tunable current value" );
    }
    operator T() const { return _value; }
    T operator() () const { return _value; }
    T set(T value) { _value = value; return _value; }
    T set_default() { _value = _default; return _value; }    
  private:
    T const _default;
    T _value;
    char const * _name;
    Item(cli::Executor * cli_exec):_default(0), _name(0)
    {
      our_cli_exec() = cli_exec;
    }
    cli::Executor * & our_cli_exec()
    {
      // Nasty, but convenient
      static cli::Executor * cli_exec;
      return cli_exec;
    }
    void deferred_register()
    {
      static std::function<void()> regchain;
      std::function<void()> rc_copy = regchain;
      auto df = [this, rc_copy]() {
        register_cmds(*our_cli_exec());
        if (rc_copy)
          rc_copy();
      };
      if (_name)
        regchain = df;
      if (our_cli_exec() && regchain) {
        regchain();
        regchain = std::function<void()>();
      }
    }
    friend void set_cli_executor(cli::Executor & cli_exec);
};

inline void set_cli_executor(cli::Executor & cli_exec)
{
  // ToDo: this wonderful mechanism only works if you know what T you want
  Item<int16_t>(&cli_exec).deferred_register();
  Item<int32_t>(&cli_exec).deferred_register();
  Item<uint16_t>(&cli_exec).deferred_register();
}

} // tunable

#endif // TUNABLE_HPP