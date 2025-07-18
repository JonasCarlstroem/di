## Simple and lightweight Dependency Injection container for C++ 17

### Sample usage

#### Using constructor requires specifying the constructor to use by supplying the arguments to `use_constructor` in the correct order as your type expects.

```cpp 
#include <iostream>
#include <di/container>
#include <di/provider>

struct abs_logger {
    virtual void log(const char* msg) = 0;
}

struct console_logger : public abs_logger {
    console_logger() = default;
    virtual void log(const char* msg) { std::cout << msg << std::endl; }
}

class sample_service {
    abs_logger* logger_;

public:
    sample_service(abs_logger* logger) : logger_(logger) {}

    void work(const char* some_text) { logger_->log(some_text); }
}

int main(int argc, char** argv) { 
    di::provider provider = di::container()
        .register_type<abs_logger, console_logger>()
        .register_type<sample_service>()
        .user_constructor<abs_logger>()
        .build();

    sample_service* service = provider.resolve<sample_service>();
    return 0; 
}

```

#### Factories allow for slightly more dynamic construction and resolution
```cpp
#include <iostream>
#include <di/container>
#include <di/provider>
#include <logging/logger> // show-case lib
#include <logging/console> // show-case lib
#include <cli/parser> // show-case lib

// Constructors:
// logging::logger::logger();
// logging::console_logger::console_logger();
// cli::parser::parser(logging::logger*)

void register_cli_args(cli::parser& parser) {
    cli::argument some_arg1{"some_arg1", "sa1", false};
    cli::argument some_arg2{"some_arg2", "sa2", ""};

    parser.add_global_argument(some_arg1);
    parser.add_global_argument(some_arg2)
}

di::provider register_services(int argc, char** argv) {
    return di::container()
        .register_type<logging::logger, logging::console_logger>()
        .register_factory<cli::parser>(
            /* this lambda can contain arbitrary args that you have to pass to  the 'register_factory' method */
            [](di::container& c, int argc, char** argv) {
                cli::parser* p = new cli::parser(
                    c.resolve<logging::logger>()
                );

                register_cli_args(*p);

                p->parse(argc, argv);
                return p;
            },
            di::lifetime::singleton,
            argc, argv // arbitrary param count after 'lifetime'. Must match lambdas signature
        )
        .build();
}

int main(int argc, char** argv) {
    di::provider provider = register_services(argc, argv);
    cli::parser* parser = provider.resolve<cli::parser>();
    //...

    return 0;
}

```