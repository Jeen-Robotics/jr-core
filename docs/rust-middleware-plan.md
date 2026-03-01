# Rust Middleware Migration Plan

## Цель

Перенести pub/sub middleware на Rust с сохранением C++ биндингов для обратной совместимости.

## Текущее состояние

Middleware (`core/middleware/`) на C++17:
- `Middleware` — брокер сообщений
- `Publisher<T>` — типизированный publisher с protobuf
- `Subscription` — RAII подписка
- `Node` — базовый класс для компонентов
- `InProcessMiddleware` — реализация с фоновыми dispatcher threads
- Keep-latest policy (queue size = 1)

## Выбор инструментов

### Для биндингов Rust ↔ C++

**[CXX](https://cxx.rs/)** — рекомендуемый выбор:
- ✅ Type-safe, zero-overhead FFI
- ✅ Поддержка `std::string`, `std::vector`, `unique_ptr`, `Box`
- ✅ Двусторонние вызовы (Rust → C++ и C++ → Rust)
- ✅ Compile-time проверка совместимости типов
- ✅ Активно поддерживается

Альтернативы (не рекомендую):
- `cbindgen` — только генерация C headers, нет type safety
- `autocxx` — автоматический парсинг C++ headers, но менее стабилен

### Для CMake интеграции

**[Corrosion](https://github.com/corrosion-rs/corrosion)** v0.6+:
- ✅ FetchContent интеграция
- ✅ `corrosion_import_crate()` для импорта Rust crates
- ✅ Автоматическая линковка static/shared libs
- ✅ Кросс-компиляция (Android!)

## Структура проекта

```
jr-core/
├── core/
│   ├── middleware/           # Существующий C++ код (НЕ удаляем)
│   │   ├── include/
│   │   ├── src/
│   │   └── test/
│   │
│   └── middleware_rs/        # Новый Rust модуль
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs        # Rust pub/sub implementation
│       │   ├── channel.rs    # MPMC channel (crossbeam или flume)
│       │   ├── topic.rs      # Topic registry
│       │   └── ffi.rs        # CXX bridge definitions
│       ├── include/
│       │   └── middleware_rs.hpp  # Generated C++ header
│       └── tests/
│           ├── rust_tests.rs
│           └── cpp_integration_test.cpp
```

## Фазы реализации

### Фаза 1: Базовая инфраструктура

1. **Создать Rust crate** `middleware_rs`:
   ```toml
   [package]
   name = "middleware_rs"
   version = "0.1.0"
   edition = "2021"

   [lib]
   crate-type = ["staticlib", "rlib"]

   [dependencies]
   # Async runtime
   tokio = { version = "1", features = ["rt-multi-thread", "sync", "macros"] }
   
   # Zero-copy channels
   async-channel = "2.0"
   
   # Protobuf (compatible with C++ protobuf wire format)
   prost = "0.13"
   prost-types = "0.13"
   
   # FFI
   cxx = "1.0"
   
   # Concurrent collections
   dashmap = "6"
   
   # Fast allocator (optional, for performance)
   mimalloc = { version = "0.1", optional = true }

   [build-dependencies]
   cxx-build = "1.0"
   prost-build = "0.13"
   ```

2. **Интегрировать Corrosion в CMakeLists.txt**:
   ```cmake
   include(FetchContent)
   FetchContent_Declare(
     Corrosion
     GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
     GIT_TAG v0.6
   )
   FetchContent_MakeAvailable(Corrosion)

   corrosion_import_crate(MANIFEST_PATH core/middleware_rs/Cargo.toml)
   ```

3. **Проверить сборку** (Rust компилируется, линкуется с C++)

### Фаза 2: Rust Pub/Sub Core

Реализовать на Rust с tokio + zero-copy:

```rust
// src/lib.rs
use std::sync::Arc;
use dashmap::DashMap;
use tokio::sync::broadcast;

pub struct Middleware {
    topics: DashMap<String, TopicChannel>,
    runtime: tokio::runtime::Handle,
}

struct TopicChannel {
    sender: broadcast::Sender<Arc<dyn std::any::Any + Send + Sync>>,
}

impl Middleware {
    pub fn new() -> Self { ... }
    
    /// Zero-copy publish — Arc передаётся без копирования
    pub async fn publish<T: Send + Sync + 'static>(
        &self, 
        topic: &str, 
        msg: Arc<T>
    ) {
        if let Some(channel) = self.topics.get(topic) {
            let _ = channel.sender.send(msg);
        }
    }
    
    /// Subscribe с типизированным receiver
    pub async fn subscribe<T: Send + Sync + 'static>(
        &self, 
        topic: &str
    ) -> broadcast::Receiver<Arc<T>> {
        // ...
    }
}

// Пример использования
async fn example() {
    let mw = Middleware::new();
    
    // Publisher
    let frame = Arc::new(CameraFrame { data: vec![1,2,3] });
    mw.publish("/camera", frame).await;
    
    // Subscriber — получает тот же Arc (zero-copy!)
    let mut sub = mw.subscribe::<CameraFrame>("/camera").await;
    while let Ok(frame) = sub.recv().await {
        println!("Got frame with {} bytes", frame.data.len());
    }
}
```

**Ключевые особенности:**
- **tokio::sync::broadcast** — multi-producer multi-consumer async channel
- **Arc<T>** — zero-copy передача между потоками
- **DashMap** — concurrent hashmap без глобального lock
- **Type erasure** через `Any` + downcast для динамических подписок

### Фаза 3: CXX Bridge

```rust
// src/ffi.rs
#[cxx::bridge]
mod ffi {
    extern "Rust" {
        type RustMiddleware;
        type Subscription;

        fn create_middleware() -> Box<RustMiddleware>;
        fn publish(mw: &RustMiddleware, topic: &str, payload: &[u8]);
        fn subscribe(mw: &mut RustMiddleware, topic: &str) -> Box<Subscription>;
        fn unsubscribe(sub: &mut Subscription);
        fn try_recv(sub: &Subscription) -> Result<Vec<u8>>;
    }
}
```

### Фаза 4: C++ Wrapper

Тонкая C++ обёртка для совместимости с существующим API:
```cpp
// include/middleware_rs.hpp
namespace jr::mw::rs {

class RustMiddleware {
public:
    void publish(const std::string& topic, const std::string& payload);
    Subscription subscribe(const std::string& topic);
private:
    rust::Box<::RustMiddleware> inner_;
};

} // namespace jr::mw::rs
```

### Фаза 5: Тесты

**Rust тесты** (`cargo test`):
```rust
#[test]
fn test_publish_subscribe() {
    let mw = RustMiddleware::new();
    let sub = mw.subscribe("/test");
    mw.publish("/test", b"hello");
    assert_eq!(sub.recv().unwrap(), b"hello");
}
```

**C++ интеграционные тесты** (GTest):
```cpp
TEST(RustMiddleware, PublishSubscribe) {
    auto mw = jr::mw::rs::create_middleware();
    auto sub = mw->subscribe("/test");
    mw->publish("/test", "hello");
    EXPECT_EQ(sub->try_recv(), "hello");
}
```

## Решения

### 1. Zero-copy межпоточная передача (без сериализации)

**Подход: `Arc<T>` для in-process**

```rust
// Публикация — создаём Arc один раз
pub async fn publish<T: Send + Sync + 'static>(&self, topic: &str, msg: Arc<T>)

// Подписка — получаем тот же Arc (zero-copy!)
pub async fn subscribe<T>(&self, topic: &str) -> Receiver<Arc<T>>
```

**Гибридная архитектура:**
```
┌─────────────────────────────────────────────────────────┐
│                    Rust Middleware                       │
├─────────────────────────────────────────────────────────┤
│  Rust ↔ Rust:   Arc<T> напрямую (zero-copy)             │
│  C++ → Rust:    Serialized bytes через FFI → prost      │
│  To disk/net:   Сериализация в protobuf wire format     │
└─────────────────────────────────────────────────────────┘
```

**Типы сообщений:**
- **prost** генерирует Rust structs из `.proto` файлов
- Совместимость с C++ protobuf wire format
- `Arc`-wrapping layer для zero-copy передачи

### 2. Async Runtime

**tokio** с самого начала:
```rust
#[tokio::main]
async fn main() {
    let mw = RustMiddleware::new();
    
    let sub = mw.subscribe::<CameraFrame>("/camera/frame").await;
    
    tokio::spawn(async move {
        while let Some(frame) = sub.recv().await {
            process(frame).await;
        }
    });
}
```

**Каналы:** `tokio::sync::broadcast` или `async-channel`

### 3. Android

Второстепенный приоритет. Настроим после desktop версии:
- Rust target: `aarch64-linux-android`
- Corrosion поддерживает кросс-компиляцию
- Интеграция с существующим android CMake preset

## Timeline

| Фаза | Описание | Оценка |
|------|----------|--------|
| 1 | Инфраструктура (Cargo + CMake) | 1-2 дня |
| 2 | Rust pub/sub core | 2-3 дня |
| 3 | CXX bridge | 1-2 дня |
| 4 | C++ wrapper | 1 день |
| 5 | Тесты | 1-2 дня |

**Итого: ~7-10 дней**

## Риски

1. **CXX ограничения** — не все C++ типы поддерживаются, может потребоваться адаптация
2. **tokio runtime в staticlib** — нужно правильно инициализировать runtime из C++
3. **Protobuf совместимость** — prost использует тот же wire format, но нужно проверить edge cases
4. **Android NDK** (низкий приоритет) — Rust + NDK требует правильной конфигурации toolchain

## Следующий шаг

✅ План утверждён!

Начинаем с **Фазы 1**:
1. Создать `core/middleware_rs/` с Cargo.toml
2. Добавить Corrosion в CMakeLists.txt
3. Написать минимальный CXX bridge
4. Проверить что Rust компилируется и линкуется с C++ проектом

---

*Последнее обновление: 2026-03-01*
