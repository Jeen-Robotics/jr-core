# Параллельные задачи для Rust Middleware

## Граф зависимостей

```
                    ┌─────────────────┐
                    │   PHASE 1       │
                    │  (параллельно)  │
                    └────────┬────────┘
                             │
         ┌───────────────────┼───────────────────┐
         ▼                   ▼                   ▼
   ┌───────────┐      ┌───────────┐      ┌───────────┐
   │  AGENT A  │      │  AGENT B  │      │  AGENT C  │
   │ Rust Core │      │   Build   │      │   Proto   │
   └─────┬─────┘      └─────┬─────┘      └─────┬─────┘
         │                   │                   │
         └───────────────────┼───────────────────┘
                             ▼
                    ┌─────────────────┐
                    │   PHASE 2       │
                    │ (после Phase 1) │
                    │   CXX Bridge    │
                    └────────┬────────┘
                             ▼
                    ┌─────────────────┐
                    │   PHASE 3       │
                    │  C++ Wrapper +  │
                    │     Tests       │
                    └─────────────────┘
```

---

## AGENT A: Rust Core Implementation

**Цель:** Реализовать pub/sub middleware на Rust с tokio

**Входные данные:** Нет зависимостей, можно начинать сразу

**Задачи:**
1. Создать `core/middleware_rs/Cargo.toml`
2. Создать структуру файлов:
   - `src/lib.rs` — публичный API
   - `src/middleware.rs` — основная структура Middleware
   - `src/topic.rs` — TopicRegistry с DashMap
   - `src/channel.rs` — типизированные broadcast каналы
3. Реализовать:
   ```rust
   pub struct Middleware { ... }
   impl Middleware {
       pub fn new() -> Self;
       pub async fn publish<T>(&self, topic: &str, msg: Arc<T>);
       pub async fn subscribe<T>(&self, topic: &str) -> Receiver<Arc<T>>;
   }
   ```
4. Написать unit тесты в `src/lib.rs` или `tests/`

**Выходные данные:**
- Рабочий Rust crate с `cargo test` passing
- Файл `core/middleware_rs/Cargo.toml`

**Критерий готовности:**
```bash
cd core/middleware_rs && cargo test
# Все тесты проходят
```

---

## AGENT B: Build System Integration

**Цель:** Интегрировать Rust crate в CMake через Corrosion

**Входные данные:** Нужно знать путь к Cargo.toml (от Agent A)

**Задачи:**
1. Добавить Corrosion в корневой `CMakeLists.txt`:
   ```cmake
   include(FetchContent)
   FetchContent_Declare(
     Corrosion
     GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
     GIT_TAG v0.6
   )
   FetchContent_MakeAvailable(Corrosion)
   ```

2. Создать `core/middleware_rs/CMakeLists.txt`:
   ```cmake
   corrosion_import_crate(MANIFEST_PATH Cargo.toml)
   # Экспортировать таргет middleware_rs
   ```

3. Добавить в `core/CMakeLists.txt`:
   ```cmake
   add_subdirectory(middleware_rs)
   ```

4. Проверить что пустой Rust crate компилируется при `cmake --build`

**Выходные данные:**
- Обновлённые CMakeLists.txt файлы
- Rust crate собирается вместе с C++ проектом

**Критерий готовности:**
```bash
cmake --preset dev && cmake --build build
# Rust crate скомпилирован, линковка успешна
```

---

## AGENT C: Proto Generation Setup

**Цель:** Настроить prost для генерации Rust типов из .proto файлов

**Входные данные:** 
- Существующие .proto файлы (jr_msgs)
- Cargo.toml от Agent A

**Задачи:**
1. Найти .proto файлы проекта (jr_msgs Conan package или локальные)

2. Создать `core/middleware_rs/build.rs`:
   ```rust
   fn main() {
       prost_build::compile_protos(
           &["proto/camera_frame.proto"],
           &["proto/"]
       ).unwrap();
   }
   ```

3. Добавить proto файлы или настроить путь к jr_msgs

4. Проверить что `cargo build` генерирует Rust structs

5. Написать тест совместимости wire format:
   ```rust
   #[test]
   fn test_wire_format_compatibility() {
       // Сериализовать в Rust, проверить что C++ может прочитать
   }
   ```

**Выходные данные:**
- `build.rs` с prost конфигурацией
- Сгенерированные Rust типы из .proto
- Тест совместимости

**Критерий готовности:**
```bash
cd core/middleware_rs && cargo build
# proto types сгенерированы в target/
```

---

## Порядок запуска

```
Время  ─────────────────────────────────────────────►

       ┌──────────────────────────────────────┐
       │ AGENT A: Rust Core                   │
       │ (может начать сразу)                 │
       └──────────────────────────────────────┘
       
       ┌──────────────────────────────────────┐
       │ AGENT B: Build System                │
       │ (ждёт Cargo.toml от A, ~5 минут)     │
       └──────────────────────────────────────┘
       
       ┌──────────────────────────────────────┐
       │ AGENT C: Proto Generation            │
       │ (ждёт Cargo.toml от A, ~5 минут)     │
       └──────────────────────────────────────┘
       
                              │
                              ▼
                    ┌─────────────────┐
                    │ SYNC POINT      │
                    │ Все 3 готовы    │
                    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ PHASE 2:        │
                    │ CXX Bridge      │
                    │ (я делаю сам    │
                    │  или новый      │
                    │  sub-agent)     │
                    └─────────────────┘
```

---

## Стратегия оркестрации

1. **Я (главный агент)** создаю минимальный `Cargo.toml` сразу
2. Запускаю **3 sub-agents параллельно** с их задачами
3. Мониторю прогресс через `/tasks`
4. Когда все завершат — делаю **sync point** проверку
5. Начинаю Phase 2 (CXX Bridge) — либо сам, либо новый sub-agent

---

## Команды для запуска

```bash
# После создания Cargo.toml запускаю:

claude --remote "Execute task A from docs/rust-mw-parallel-tasks.md: Implement Rust Core"
claude --remote "Execute task B from docs/rust-mw-parallel-tasks.md: Build System Integration"  
claude --remote "Execute task C from docs/rust-mw-parallel-tasks.md: Proto Generation Setup"
```

---

Готов запускать?
