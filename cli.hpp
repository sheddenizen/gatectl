#ifndef CLI_HPP
#define CLI_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <functional>
#include <tuple>

namespace cli
{



namespace aux {
    using std::istream;
    using std::istringstream;
    using std::ostringstream;
    using std::ostream;
    using std::tuple;
    using std::true_type;
    using std::false_type;
    using std::string;

    template <int N>
    using is_zero = typename std::conditional<N == 0, true_type, false_type>::type;

    template <int N, typename... VN>
    int read_tuple(istream & is, true_type, tuple<VN...> &t) { return N; }

    template <int N, typename... VN>
    int read_tuple(istream & is, false_type, tuple<VN...> &t)
    {
      is >> std::get<sizeof...(VN)-N>(t);
      if (is.fail())
        return N;
      return read_tuple<N-1>(is, is_zero<N-1>(), t);
    }

    template <int N, typename... VN>
    void write_tuple(ostream & os, true_type, tuple<VN...> const &t) {}

    template <int N, typename... VN>
    void write_tuple(ostream & os, false_type, tuple<VN...> const &t)
    {
      os << std::get<sizeof...(VN)-N>(t);
      if (os.good())
        write_tuple<N-1>(os, is_zero<N-1>(), t);
    }

    template <typename T>
    struct fnx;

    template <typename R, typename... Args>
    struct fnx<std::function<R (Args...)>> {
        using arg_tuple = tuple<Args...>;
        using rtype = R;
    };


    class Cmd
    {
      public:
        explicit Cmd(string const & cmdname, string const & cmdhelp = {})
          : _cmdname(cmdname)
          , _cmdhelp(cmdhelp)
        {}
        virtual ~Cmd() {};
        string operator() (istringstream & is) const 
        {
          return exec(is);
        }
        string const & help() const { return _cmdhelp; }
        Cmd * next() { return _next_cmd; }
        void next(Cmd * next_cmd) { _next_cmd = next_cmd; }
        string const & cmdname() const { return _cmdname; }
        string const & cmdhelp() const { return _cmdhelp; }
      protected:
        virtual string exec(std::istringstream & is) const = 0;
      private:
        string _cmdname;
        string _cmdhelp;
        Cmd * _next_cmd = 0;
    };

    template<typename Signature, typename ArgTuple>
    class CmdHandler : public Cmd
    {
      public:
        using Fn = std::function<Signature>;
        CmdHandler(string const & cmdname, Fn fn, string const & cmdhelp = {})
          : Cmd(cmdname, cmdhelp)
          , _fn(fn)
        {}
        virtual string exec(istringstream & is) const
        {
          using Tuple = typename fnx<Fn>::arg_tuple;
          Tuple t;
          constexpr auto TSize = std::tuple_size<Tuple>::value;
          int n = aux::read_tuple<TSize>(is, aux::is_zero<TSize>(), t);
          ostringstream os;
          if (!is.fail()) {
            auto result = std::apply(_fn, t);
            os << cmdname() << ": " << result;
          } else {
            os << "Error: parsing parameter " << (TSize - n + 1) << " for command, " << cmdname();
          }
          return os.str();
        }
      private:
        Fn _fn;
    };

  } // namespace aux

  template <typename... VN>
  std::istream & operator >> (std::istream & is, std::tuple<VN...> &t)
  {
    aux::read_tuple<sizeof...(VN)>(is, aux::is_zero<sizeof...(VN)>(), t);
    return is;
  }

  template <typename... VN>
  std::ostream & operator << (std::ostream & os, std::tuple<VN...> const &t)
  {
    aux::write_tuple<sizeof...(VN)>(os, aux::is_zero<sizeof...(VN)>(), t);
    return os;
  }

  class Executor
  {
    public:
      Executor() {
        add_command("help", std::function([this]{return this->help();}), "Help on available commands");
      }
      template<typename Signature>
      void add_command(std::string const & command, std::function<Signature> fn, std::string const & cmdhelp = {})
      {
        using Fn = std::function<Signature>;
        using Tuple = typename aux::fnx<Fn>::arg_tuple;

        aux::Cmd * cmd = new aux::CmdHandler<Signature, Tuple>(command, fn, cmdhelp);
        add_tail(cmd);
      }
      std::string operator() (std::string const & line) {
        std::istringstream is(line);
        std::string cmdname;
        is >> cmdname;

        aux::Cmd * cmd= _head_cmd;
        for (; cmd && cmdname != cmd->cmdname() ; cmd = cmd->next());
        std::string result;
        if (cmd) {
          return (*cmd)(is);
        } else {
          return std::string("Command, ") + cmdname + " not found";
        }
      }

    private:
      aux::Cmd * _head_cmd = 0;
      void add_tail(aux::Cmd * cmd)
      {
        if (_head_cmd == 0) {
          _head_cmd = cmd;
          return;
        }
        aux::Cmd * tail = _head_cmd;
        while (tail->next()) {
          tail = tail->next();
        }
        tail->next(cmd);
      }
      std::string help()
      {
        std::ostringstream os;
        os << "Available commands:" << std::endl;
        for (aux::Cmd * cmd = _head_cmd; cmd ; cmd = cmd->next()) {
          os << '\t' << cmd->cmdname() << ": \t" << cmd->cmdhelp() << std::endl;
        }
        return os.str();
      }
  };

} // namespace cli


#endif // CLI_HPP
