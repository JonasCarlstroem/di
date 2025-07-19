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

#define typeof(Type) std::type_index(typeid(Type))

namespace di {

class container;
class provider;

template <typename TInterface, typename... TUserArgs>
struct factory_function {
    using type = TInterface*(
        provider&,
        TUserArgs...
    );
};

template <typename TInterface, typename... TUserArgs>
using factory_function_t = typename factory_function<TInterface, TUserArgs...>::type;

enum class lifetime { singleton, transient };

#ifdef NDEBUG
#define DI_THROW_OR_ASSERT(msg) throw std::runtime_error(msg)
#else
#define DI_THROW_OR_ASSERT(msg) assert(false && msg)
#endif

template <typename TConcrete>
class registration_builder {
    container& parent_;

  public:
    explicit registration_builder(container& parent)
        : parent_(parent) {}

    registration_builder(const registration_builder&)                = default;
    registration_builder(registration_builder&&)                     = default;
    registration_builder& operator=(const registration_builder&)     = default;
    registration_builder& operator=(registration_builder&&) noexcept = default;

    template <typename UConcrete>
    auto& register_type(lifetime lt = lifetime::singleton) {
        if (parent_.is_built_) throw std::runtime_error("Provider has already been built.");
        return parent_.template register_type<UConcrete>(lt);
    }

    template <typename... TArgs>
    container& use_constructor() {
        if (parent_.is_built_) throw std::runtime_error("Provider has already been built.");
        return parent_.template use_constructor<TConcrete, TArgs...>();
    }

    template <
        typename TInterface,
        typename... TArgs>
    container& register_factory(
        factory_function_t<
            TInterface,
            TArgs...> factory_fn,
        lifetime lt = lifetime::singleton,
        TArgs... args
    ) {
        if (parent_.is_built_) throw std::runtime_error("Provider has already been built.");
        return parent_.template register_factory<TInterface, TArgs...>(factory_fn, lt, args...);
    }

}; // class registration_builder<>

class container : public std::enable_shared_from_this<container> {
    template<typename TConcrete>
    friend class registration_builder;

    friend provider;

  public:
    container() = default;

    ~container() { shutdown(); }

    container(const container&)            = delete;
    container& operator=(const container&) = delete;

    container(container&&)                 = delete;
    container& operator=(container&&)      = delete;

    // Register interface->implementation
    template <
        typename TInterface,
        typename TImpl>
    registration_builder<TImpl>& register_type(lifetime lt = lifetime::singleton) {
        if (is_built_) throw std::runtime_error("Provider has already been built.");

        static_assert(std::is_base_of_v<TInterface, TImpl>, "TImpl must derive from TInterface");
        register_impl<TInterface, TImpl>(lt);
        return registration_builder<TImpl>(*this);
    }

    // Register concrete type
    template <typename TConcrete>
    registration_builder<TConcrete>& register_type(lifetime lt = lifetime::singleton) {
        if (is_built_) throw std::runtime_error("Provider has already been built.");

        static_assert(
            !std::is_abstract_v<TConcrete>,
            "Cannot register abstract type without interface"
        );

        register_impl<TConcrete, TConcrete>(lt);
        return registration_builder<TConcrete>(*this);
    }

    // Pick constructor to use (I.E, some_ns::some_cls::some_cls(logger*, parser*) <- = use_constructor<some_ns::some_cls, logger*, parser*>())
    template <
        typename TConcrete,
        typename... TArgs>
    container& use_constructor() {
        if (is_built_) throw std::runtime_error("Provider has already been built.");

        constructor_overrides_[typeof(TConcrete)] = [](provider& p) {
            return std::apply(
                [&p](auto&&... args) -> void* {
                    return new TConcrete(p.container_->resolve_dependency<std::decay_t<decltype(args)>>()...);
                },
                p.container_->resolve_tuple<std::tuple<TArgs...>>()
            );
        };
        return *this;
    }

    // Register factory for TInterface

    /// <summary>
    /// Register factory for TInterface
    /// </summary>
    /// <typeparam name="TInterface">Service type</typeparam>
    /// <typeparam name="...TUserArgs">Optional arguments for factory</typeparam>
    /// <param name="factory_fn">Factory for type</param>
    /// <param name="lt"></param>
    /// <param name="...user_args"></param>
    /// <returns></returns>
    template <
        typename TInterface,
        typename... TUserArgs>
    container& register_factory(
        factory_function_t<
            TInterface,
            TUserArgs...> factory_fn,
        lifetime lt = lifetime::singleton,
        TUserArgs... user_args
    ) {
        if (is_built_) throw std::runtime_error("Provider has already been built.");

        static_assert(
            std::is_invocable_r_v<
                TInterface*,
                factory_function_t<TInterface, TUserArgs...>,
                provider&,
                TUserArgs...>,
            "Factory function must take (container&, user_args...) and return TInterface*"
        );

        factories_[typeof(TInterface)] = {
          [factory_fn,
           user_args_tuple =
               std::make_tuple(std::forward<TUserArgs>(user_args)...)](provider& p) -> void* {
              return p.container_->invoke_factory<TInterface>(factory_fn, user_args_tuple);
          },
          lt
        };

        return *this;
    }

    std::shared_ptr<provider> build() {
        if (is_built_) throw std::runtime_error("Provider has already been built.");

        is_built_ = true;

        auto self = shared_from_this();
        return std::make_shared<provider>(self);
    }

    void shutdown() {
        if (!mutex_) return;

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

  private:
    struct service_descriptor {
        std::function<void*(provider&)> factory;
        lifetime life = lifetime::singleton;
    };

    std::unordered_map<std::type_index, service_descriptor> factories_ {};
    std::unordered_map<std::type_index, void*> singletons_ {};
    std::unordered_map<std::type_index, std::function<void*(provider&)>> constructor_overrides_ {};
    std::vector<void*> transients_;

    std::shared_ptr<std::recursive_mutex> mutex_ = std::make_shared<std::recursive_mutex>();
    bool is_built_                               = false;

    template <
        typename TInterface,
        typename TImpl>
    void register_impl(lifetime lt) {
        factories_[typeof(TInterface)] = service_descriptor {
          [](provider& p) -> void* {
              return static_cast<TInterface*>(p.container_->create_instance<TImpl>());
          },
          lt
        };
    }

    template <typename T>
    T* resolve_impl() {
        if (!is_built_) throw std::runtime_error("Provider has not yet been built.");

        auto it = factories_.find(typeof(T));
        if (it == factories_.end()) {
            DI_THROW_OR_ASSERT("Type not registered");
        }

        auto& descriptor = it->second;
        if (descriptor.life == lifetime::singleton) {
            auto singleton_it = singletons_.find(typeof(T));
            if (singleton_it == singletons_.end()) {
                auto instance          = (T*)(descriptor.factory(*provider_instance));
                singletons_[typeof(T)] = instance;
                return instance;
            }
            return static_cast<T*>(singleton_it->second);
        } else {
            T* instance = (T*)(descriptor.factory(*provider_instance));
            transients_.push_back(instance);
            return instance;
        }
    }

    template <typename T>
    T* create_instance() {
        if (!is_built_) throw std::runtime_error("Provider has not yet been built!");

        auto override_it = constructor_overrides_.find(typeof(T));
        if (override_it != constructor_overrides_.end()) {
            auto obj = override_it->second(*provider_instance);
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

    template <
        typename T,
        typename Tuple>
    T* construct_with_tuple() {
        return std::apply(
            [this](auto&&... args) {
                return new T(resolve_dependency<std::decay_t<decltype(args)>>()...);
            },
            resolve_tuple<Tuple>()
        );
    }

    template <
        typename T,
        typename TupleList,
        std::size_t Index = 0>
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

    template <
        typename Tuple,
        std::size_t... Is>
    auto resolve_tuple_impl(std::index_sequence<Is...>) {
        return std::make_tuple(resolve_dependency<std::tuple_element_t<Is, Tuple>>()...);
    }

    template <typename Tuple>
    auto resolve_tuple() {
        return resolve_tuple_impl<Tuple>(std::make_index_sequence<std::tuple_size_v<Tuple>> {});
    }

    template <
        typename TInterface,
        typename TFactory,
        typename TUserArgs>
    void* invoke_factory(
        TFactory& f,
        TUserArgs& user_args
    ) {
        if (!is_built_) throw std::runtime_error("Provider has not yet been built.");
        auto result = std::apply(
            [&](auto&&... args) {
                return f(*provider_instance, std::forward<decltype(args)>(args)...);
            },
            user_args
        );
        static_assert(
            std::is_pointer_v<decltype(result)>,
            "Factory function must return a pointer"
        );
        static_assert(
            std::is_convertible_v<decltype(result), TInterface*>,
            "Return type must match interface"
        );

        return static_cast<void*>(result);
    }

    template <
        typename TFactory,
        typename Tuple,
        size_t... Is>
    auto apply_with_index_impl(
        TFactory& f,
        Tuple& t,
        std::index_sequence<Is...>
    ) {
        return f(std::get<Is>(t)...);
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

    template <
        typename Tuple,
        std::size_t... Is>
    auto resolve_dependencies_impl(std::index_sequence<Is...>) {
        return std::make_tuple(resolve_dependency<std::tuple_element_t<Is, Tuple>>()...);
    }

    template <typename TFactory>
    auto resolve_dependencies() {
        using arg_types = typename function_traits<std::decay_t<TFactory>>::args_tuple;
        return resolve_tuple<arg_types>();
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
            std::tuple<>,
            std::tuple<T*>,
            std::tuple<T*, T*>,
            std::tuple<T*, T*, T*>,
            std::tuple<T*, T*, T*, T*>,
            std::tuple<T*, T*, T*, T*, T*>>;

        template <
            typename Tuple,
            std::size_t... Is>
        static constexpr bool is_constructible_from_tuple_impl(std::index_sequence<Is...>) {
            return is_constructible_with<std::tuple_element_t<Is, Tuple>...>();
        }

        template <typename Tuple>
        static constexpr bool is_constructible_from_tuple() {
            return is_constructible_from_tuple_impl<Tuple>(
                std::make_index_sequence<std::tuple_size_v<Tuple>> {}
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
                typename prepend<typename filter_ctors<type_list<Tail...>>::type, Head>::type,
                typename filter_ctors<type_list<Tail...>>::type>;
        };

        using filtered_ctors = typename filter_ctors<possible_ctors>::type;

      public:
        using largest_ctor_args = typename find_largest_ctor<filtered_ctors>::type;
    }; // class constructor_traits<>

    template <typename>
    struct function_traits;

    template <typename R, typename... Args>
    struct function_traits<R(Args...)> {
        using result_type = R;

        template <size_t I>
        using arg = std::tuple_element_t<I, std::tuple<Args...>>;

        template <size_t From, size_t Count>
        using args                    = std::tuple_element_t<From, std::tuple<Args...>>;

        using args_tuple              = std::tuple<std::decay_t<Args>...>;

        static constexpr size_t arity = sizeof...(Args);
    }; // struct function_traits<>

    template <typename R, typename... Args>
    struct function_traits<R (*)(Args...)> : function_traits<R(Args...)> {};

    template <typename C, typename R, typename... Args>
    struct function_traits<R (C::*)(Args...) const> : function_traits<R(Args...)> {};

    template <typename T>
    struct function_traits : function_traits<decltype(&T::operator())> {};

}; // class container

} // namespace di
