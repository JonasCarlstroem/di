#pragma once
#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <unordered_map>

namespace di {

template <bool>
class container;

enum class lifetime { singleton, transient };

#ifdef NDEBUG
#define DI_THROW_OR_ASSERT(msg) throw std::runtime_error(msg)
#else
#define DI_THROW_OR_ASSERT(msg) assert(false && msg)
#endif

template <typename TConcrete, bool auto_construct>
class registration_builder : public container<auto_construct> {
    container<auto_construct>& parent_;

  public:
    explicit registration_builder(container<auto_construct>& parent)
        : parent_(parent) {
        container<>::mutex_ = parent.mutex_;
    }

    registration_builder(const registration_builder&)                = default;
    registration_builder(registration_builder&&)                     = default;
    registration_builder& operator=(const registration_builder&)     = default;
    registration_builder& operator=(registration_builder&&) noexcept = default;

    template <typename UConcrete>
    auto& register_type(lifetime lt = lifetime::singleton) {
        return parent_.template register_type<UConcrete>(lt);
    }

    template <typename... TArgs>
    container<auto_construct>& use_constructor() {
        parent_.template use_constructor<TConcrete, TArgs...>();
        return parent_;
    }
};

template <bool auto_construct = false>
class container {
  public:
    container() = default;
    ~container() { shutdown(); }

    container(const container&)            = delete;
    container& operator=(const container&) = delete;

    container(container&&) noexcept        = default;
    container& operator=(container&&)      = default;

    // Register interface->implementation
    template <typename TInterface, typename TImpl>
    registration_builder<TImpl, auto_construct>&
    register_type(lifetime lt = lifetime::singleton) {
        static_assert(
            std::is_base_of_v<TInterface, TImpl>,
            "TImpl must derive from TInterface"
        );
        register_impl<TInterface, TImpl>(lt);
        return registration_builder<TImpl, auto_construct>(*this);
    }

    // Register concrete type
    template <typename TConcrete>
    registration_builder<TConcrete, auto_construct>&
    register_type(lifetime lt = lifetime::singleton) {
        static_assert(
            !std::is_abstract_v<TConcrete>,
            "Cannot register abstract type without interface"
        );
        register_impl<TConcrete, TConcrete>(lt);
        return registration_builder<TConcrete, auto_construct>(*this);
    }

    // Explicit constructor override
    template <typename TConcrete, typename... TArgs>
    container& use_constructor() {
        constructor_overrides_[std::type_index(typeid(TConcrete))] = [this]() {
            return std::apply(
                [this](auto&&... args) -> void* {
                    return new TConcrete(
                        resolve_dependency<std::decay_t<decltype(args)>>()...
                    );
                },
                resolve_tuple<std::tuple<TArgs...>>()
            );
        };
        return *this;
    }

    template <typename TInterface>
    container& register_factory(
        std::function<void*()> factory_fn, lifetime lt = lifetime::singleton
    ) {
        factories_[std::type_index(typeid(TInterface))] = {factory_fn, lt};
        return *this;
    }

    // Resolve ptr
    template <typename T>
    T* resolve() {
        std::lock_guard<std::recursive_mutex> lock(*mutex_);
        return resolve_impl<T>();
    }

    // Convenience: resolve raw pointer
    template <typename T>
    T* resolve_ptr() {
        return resolve_impl<T>().get();
    }

    void shutdown() {
        std::lock_guard<std::recursive_mutex> lock(*mutex_);
        for (auto& [key, ptr] : singletons_) {
            delete static_cast<char*>(ptr);
        }
        singletons_.clear();

        for (auto ptr : transients_) {
            delete static_cast<char*>(ptr);
        }
        transients_.clear();
    }

  protected:
    std::shared_ptr<std::recursive_mutex> mutex_ = std::make_shared<std::recursive_mutex>();

  private:
    struct service_descriptor {
        std::function<void*()> factory;
        lifetime life = lifetime::singleton;
    };

    std::unordered_map<std::type_index, service_descriptor> factories_;
    std::unordered_map<std::type_index, void*> singletons_;
    std::unordered_map<std::type_index, std::function<void*()>>
        constructor_overrides_;
    std::vector<void*> transients_;

    template <typename TInterface, typename TImpl>
    void register_impl(lifetime lt) {
        factories_[std::type_index(typeid(TInterface))] = {
            [this]() -> void* {
                return static_cast<TInterface*>(create_instance<TImpl>());
            },
            lt
        };
    }

    template <typename T>
    T* resolve_impl() {
        std::lock_guard<std::recursive_mutex> lock(*mutex_);

        auto it = factories_.find(std::type_index(typeid(T)));
        if (it == factories_.end()) {
            if constexpr (auto_construct && !std::is_abstract_v<T>) {
                T* instance = create_instance<T>();
                transients_.push_back(instance);
                return instance;
            }
            DI_THROW_OR_ASSERT("Type not registered");
        }

        auto& descriptor = it->second;
        if (descriptor.life == lifetime::singleton) {
            auto singleton_it = singletons_.find(std::type_index(typeid(T)));
            if (singleton_it == singletons_.end()) {
                auto instance = (T*)(descriptor.factory());
                singletons_[std::type_index(typeid(T))] = instance;
                return instance;
            }
            return static_cast<T*>(singleton_it->second);
        } else {
            T* instance = (T*)(descriptor.factory());
            transients_.push_back(instance);
            return instance;
        }
    }

    template <typename T>
    T* create_instance() {
        auto override_it =
            constructor_overrides_.find(std::type_index(typeid(T)));
        if (override_it != constructor_overrides_.end()) {
            auto obj = override_it->second();
            return static_cast<T*>(obj);
        }

        using ctor_args = typename constructor_traits<T>::largest_ctor_args;

        if constexpr (std::tuple_size_v<ctor_args> == 0) {
            if constexpr (std::is_default_constructible_v<T>) {
                return new T();
            } else {
                DI_THROW_OR_ASSERT("No suitable constructor found for type");
            }
        } else {
            return construct_with_tuple<T, ctor_args>();
        }
    }

    template <typename T, typename Tuple>
    T* construct_with_tuple() {
        return std::apply(
            [this](auto&&... args) {
                return new T(
                    resolve_dependency<std::decay_t<decltype(args)>>()...
                );
            },
            resolve_tuple<Tuple>()
        );
    }

    template <typename T>
    T* construct_with_best_ctor() {
        using ctor_list = typename constructor_traits<T>::ctor_args_list;

        return try_ctors<T, ctor_list>();
    }

    template <typename T, typename TupleList, std::size_t Index = 0>
    T* try_ctors() {
        if constexpr (Index < std::tuple_size_v<TupleList>) {
            using Args = std::tuple_element_t<Index, TupleList>;

            if constexpr (are_resolvable<Args>()) {
                return construct_with_tuple<T, Args>();
            } else {
                return try_ctors<T, TupleList, Index + 1>();
            }
        } else {
            DI_THROW_OR_ASSERT("No suitable constructor found for type");
            return nullptr;
        }
    }

    template <typename Tuple>
    constexpr bool are_resolvable() {
        return (can_resolve<std::tuple_element_t<0, Tuple>>() && ...);
    }

    template <typename U>
    constexpr bool can_resolve() {
        return std::is_default_constructible_v<U> ||
               (factories_.find(std::type_index(typeid(U))) !=
                factories_.end());
    }

    template <typename Tuple, std::size_t... Is>
    auto resolve_tuple_impl(std::index_sequence<Is...>) {
        return std::make_tuple(
            resolve_dependency<std::tuple_element_t<Is, Tuple>>()...
        );
    }

    template <typename Tuple>
    auto resolve_tuple() {
        return resolve_tuple_impl<Tuple>(
            std::make_index_sequence<std::tuple_size_v<Tuple>>{}
        );
    }

    template <typename U>
    auto resolve_dependency() {
        if constexpr (std::is_pointer_v<U>) {
            using Pointee = std::remove_pointer_t<U>;
            return resolve_impl<Pointee>(); // returns raw pointer
        } else if constexpr (std::is_reference_v<U>) {
            using RefType = std::remove_reference_t<U>;
            return *resolve_impl<RefType>(); // returns reference
        } else {
            return resolve_impl<U>(); // returns shared_ptr<T>
        }
    }

    template <typename T>
    struct deleter {
        void operator()(T* ptr) const { delete ptr; }
    };

    template <typename T>
    struct constructor_traits {
      private:
        template <typename... Args>
        static constexpr bool is_constructible_with() {
            return std::is_constructible_v<T, Args...>;
        }

        template <typename...>
        struct type_list {};

        template <typename List, typename Tuple>
        struct prepend;

        template <typename... Ts, typename... Us>
        struct prepend<type_list<Ts...>, std::tuple<Us...>> {
            using type = type_list<std::tuple<Us...>, Ts...>;
        };

        template <typename List>
        struct find_largest_ctor;

        template <>
        struct find_largest_ctor<type_list<>> {
            using type = std::tuple<>;
        };

        template <typename Head, typename... Tail>
        struct find_largest_ctor<type_list<Head, Tail...>> {
            using type = Head;
        };

        using possible_ctors = type_list<
            std::tuple<>, std::tuple<T*>, std::tuple<T*, T*>,
            std::tuple<T*, T*, T*>, std::tuple<T*, T*, T*, T*>,
            std::tuple<T*, T*, T*, T*, T*>>;

        template <typename Tuple, std::size_t... Is>
        static constexpr bool
        is_constructible_from_tuple_impl(std::index_sequence<Is...>) {
            return is_constructible_with<std::tuple_element_t<Is, Tuple>...>();
        }

        template <typename Tuple>
        static constexpr bool is_constructible_from_tuple() {
            return is_constructible_from_tuple_impl<Tuple>(
                std::make_index_sequence<std::tuple_size_v<Tuple>>{}
            );
        }

        template <typename List>
        struct filter_ctors;

        template <>
        struct filter_ctors<type_list<>> {
            using type = type_list<>;
        };

        template <typename Head, typename... Tail>
        struct filter_ctors<type_list<Head, Tail...>> {
            using type = std::conditional_t<
                is_constructible_from_tuple<Head>(),
                typename prepend<
                    typename filter_ctors<type_list<Tail...>>::type,
                    Head>::type,
                typename filter_ctors<type_list<Tail...>>::type>;
        };

        using filtered_ctors = typename filter_ctors<possible_ctors>::type;

      public:
        using largest_ctor_args =
            typename find_largest_ctor<filtered_ctors>::type;
    };
};

} // namespace di
