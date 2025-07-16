#pragma once

#include "di_container2.hpp"

namespace di {

template <bool auto_construct = false>
class provider {
  public:
    explicit provider(container<auto_construct>& cont) : container_(cont) {}

    template <typename T>
    T* get() {
        // return
    }

  private:
    container<auto_construct>& container_;

    template<typename T>
    T* resolve_impl() {
        if (auto instance = container_.try_resolve<T>()) {
            return instance;
        }

        if constexpr (auto_construct && !std::is_abstract_v<T>) {
            return create_instance<T>();
        } else {
            DI_THROW_OR_ASSERT("Type not registered and cannot auto-construct");
        }
    }

    template<typemane T>
    T* create_instance() {

    }
};

} // namespace di