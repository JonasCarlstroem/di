#pragma once

#include "di_container.hpp"

namespace di {

class provider {
    friend container;

  public:
    explicit provider(container* cont) : container_(cont) {}

    template <typename T>
    T* resolve() {
        return container_->template resolve_impl<T>();
    }

  private:
    container* container_;
};

} // namespace di