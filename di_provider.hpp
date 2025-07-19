#pragma once

#include "di_container.hpp"

namespace di {

class provider {
    friend container;
    static inline std::set<provider*> providers_;
    std::shared_ptr<container> container_;

    explicit provider(std::shared_ptr<container> cont)
        : container_(std::move(cont)) {}

    static provider& create_provider(std::shared_ptr<container> cont) {
        auto ptr = new di::provider(cont);
        providers_.insert(ptr);
        return *ptr;
    }

  public:
    ~provider() { providers_.erase(this); }

    template <typename T>
    T* resolve() {
        return container_->template resolve_impl<T>(*this);
    }
};

} // namespace di