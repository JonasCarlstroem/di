#pragma once

#include "di_container.hpp"

namespace di {

class provider {
    friend container;
    std::shared_ptr<container> container_;

  public:
    explicit provider(std::shared_ptr<container> cont) : container_(std::move(cont)) {}

    template <typename T>
    T* resolve() {
        return container_->template resolve_impl<T>();
    }
};

} // namespace di