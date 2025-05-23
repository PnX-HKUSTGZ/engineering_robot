# Engineering_robot

## 目录

1.  [引言](#1-引言)
2.  [通用格式化](#2-通用格式化)
    *   [2.1 缩进与空格](#21-缩进与空格)
    *   [2.2 行长度](#22-行长度)
    *   [2.3 大括号 (Allman风格)](#23-大括号-allman风格)
    *   [2.4 空行](#24-空行)
    *   [2.5 函数声明与定义](#25-函数声明与定义)
    *   [2.6 指针与引用](#26-指针与引用)
3.  [命名约定](#3-命名约定)
    *   [3.1 通用原则](#31-通用原则)
    *   [3.2 文件名](#32-文件名)
    *   [3.3 命名空间](#33-命名空间)
    *   [3.4 类与结构体](#34-类与结构体)
    *   [3.5 函数与方法](#35-函数与方法)
    *   [3.6 变量](#36-变量)
    *   [3.7 常量、枚举、配置参数、静态变量](#37-常量枚举配置参数静态变量)
    *   [3.8 宏定义](#38-宏定义)
4.  [注释](#4-注释)
    *   [4.1 通用原则](#41-通用原则)
    *   [4.2 类与结构体注释](#42-类与结构体注释)
    *   [4.3 函数与方法注释](#43-函数与方法注释)
    *   [4.4 行内注释](#44-行内注释)
    *   [4.5 TODO, FIXME, NOTE](#45-todo-fixme-note)
5.  [C++ 特性使用](#5-c-特性使用)
    *   [5.1 `auto`关键字](#51-auto关键字)
    *   [5.2 智能指针](#52-智能指针)
    *   [5.3 `nullptr`](#53-nullptr)
    *   [5.4 `const` 正确性](#54-const-正确性)
    *   [5.5 `override` 和 `final`](#55-override-和-final)
    *   [5.6 异常处理](#56-异常处理)
    *   [5.7 Lambda 表达式](#57-lambda-表达式)
    *   [5.8 范围 `for` 循环](#58-范围-for-循环)
    *   [5.9 初始化](#59-初始化)
6.  [ROS 2 日志规范](#6-ros-2-日志规范)
7.  [代码组织](#7-代码组织)
    *   [7.1 头文件保护](#71-头文件保护)
    *   [7.2 `include` 顺序](#72-include-顺序)
    *   [7.3 类定义顺序](#73-类定义顺序)
    *   [7.4 避免使用 `using namespace`](#74-避免使用-using-namespace)
8.  [工具](#8-工具)
    *   [8.1 格式化工具 (ClangFormat)](#81-格式化工具-clangformat)
    *   [8.2 静态分析工具 (ClangTidy, Cppcheck)](#82-静态分析工具-clangtidy-cppcheck)
9.  [版本控制](#9-版本控制)
    *   [9.1提交信息](#91提交信息)

---

## 1. 引言

本代码规范旨在提高[项目名称]代码库的可读性、可维护性和一致性。所有参与项目的开发者都应遵守此规范。

## 2. 通用格式化

### 2.1 缩进与空格

*   **缩进**: 使用 **4个空格** 进行缩进，**禁止使用制表符(Tab)**。
*   **空格**:
    *   二元操作符 (e.g., `+`, `-`, `=`, `*`, `/`, `==`, `!=`, `&&`, `||`) 两侧各加一个空格: `a = b + c;`
    *   逗号 (`,`) 和分号 (`;`) 后面加一个空格，前面不加: `foo(a, b);`
    *   控制语句关键字 (`if`, `for`, `while`, `switch`) 后面加一个空格: `if (condition)`
    *   函数调用时，函数名和左括号之间不加空格: `myFunction(arg);`
    *   单行注释 `//` 后留一个空格: `// This is a comment.`
    *   指针或引用符号 (`*`, `&`) 建议靠近类型名: `int* p;` 或 `const std::string& str;`。

### 2.2 行长度

*   每行代码长度建议不超过 **100 或 120 个字符**。
*   超过长度时，应在合适的逻辑断点处换行，新行相对上一行缩进一级（通常4个空格）。

### 2.3 大括号 (Allman风格)

*   **强制使用 Allman 风格**: 大括号独占一行，并与上一级代码对齐。
    ```cpp
    if (condition)
    {
        // code
    }
    else
    {
        // code
    }
    ```
*   对于 `if`, `for`, `while` 等控制结构，即使只有单行语句，也 **必须使用大括号**。
    ```cpp
    // Good
    if (condition)
    {
        doSomething();
    }
    ```

### 2.4 空行

*   在逻辑相关的代码块之间使用空行分隔，以提高可读性。
*   函数定义之间使用1-2个空行。
*   类定义中，`public`, `protected`, `private` 区域之间使用空行。
*   文件末尾保留一个空行。

### 2.5 函数声明与定义

*   返回值类型与函数名在同一行。
*   参数列表较长时，每个参数可以独占一行，与第一个参数对齐或缩进一级。
    ```cpp
    ReturnType longFunctionName(
        ArgumentType1 arg1,
        ArgumentType2 arg2,
        ArgumentType3 arg3);
    ```

### 2.6 指针与引用

*   `*` 或 `&` 符号建议紧靠类型名。
    ```cpp
    std::string* name_ptr;
    const cv::Mat& image_ref;
    ```

## 3. 命名约定

### 3.1 通用原则

*   **清晰性**: 名字应清晰表达其含义。
*   **一致性**: 在整个项目中保持命名风格一致。
*   **避免缩写**: 除非是广泛接受的缩写 (e.g., `num`, `id`, `url`, `ptr`)。
*   **避免使用匈牙利命名法** (e.g., `iCount`, `bIsValid`)。

### 3.2 文件名

*   使用 **小写下划线命名法 (snake_case)**。
*   头文件使用 `.hpp` 后缀。
*   源文件使用 `.cpp` 后缀。
    ```
    my_class.hpp
    my_class.cpp
    ```

### 3.3 命名空间

*   使用 **小写下划线命名法 (snake_case)**。
*   命名空间名称应具有描述性。
    ```cpp
    namespace my_project_namespace
    {
    // ...
    } // namespace my_project_namespace
    ```

### 3.4 类与结构体

*   使用 **大驼峰命名法 (PascalCase)**。
    ```cpp
    class MyClass
    {
    // ...
    };

    struct PointData
    {
    // ...
    };
    ```

### 3.5 函数与方法

*   **类方法 (公有、保护、私有)**: 使用 **小驼峰命名法 (camelCase)**。
    ```cpp
    class MyProcessor
    {
    public:
        void processImage(const cv::Mat& image);
    private:
        void calculateInternalState();
    };
    ```
*   **自由函数 (非成员函数)**: 使用 **小写下划线命名法 (snake_case)**。
    ```cpp
    cv::Mat preprocess_image(const cv::Mat& raw_image);
    ```

### 3.6 变量

*   **局部变量**: 使用 **小写下划线命名法 (snake_case)**。
    ```cpp
    int_least32_t item_count = 0;
    std::string user_name;
    ```
*   **类成员变量 (非静态)**: 使用 **小写下划线命名法，并加 `_` 后缀** (e.g., `member_variable_`)。
    ```cpp
    class MyData
    {
    private:
        int data_value_;
        std::string name_str_;
    };
    ```

### 3.7 常量、枚举、配置参数、静态变量

*   **常量 (`const`, `constexpr`)**: 使用 **全大写下划线命名法 (SCREAMING_SNAKE_CASE)**。
    ```cpp
    const double PI_VALUE = 3.14159;
    constexpr int MAX_BUFFER_SIZE = 1024;
    ```
*   **枚举类型 (enum class)**: 使用 **大驼峰命名法 (PascalCase)**。
*   **枚举值**: 使用 **全大写下划线命名法 (SCREAMING_SNAKE_CASE)**。
    ```cpp
    enum class ColorState
    {
        RED_COLOR,
        GREEN_COLOR,
        BLUE_COLOR
    };
    ```
*   **从配置文件中读取的运行时参数 (通常存储在类成员中)**: 使用 **全大写下划线命名法 (SCREAMING_SNAKE_CASE)**。
    ```cpp
    class ConfigurableComponent
    {
    public:
        void loadConfig(const YAML::Node& config)
        {
            SCAN_ANGLE_MIN_ = config["scan_angle_min"].as<double>();
            ENABLE_FEATURE_X_ = config["enable_feature_x"].as<bool>();
        }
    private:
        double SCAN_ANGLE_MIN_;
        bool ENABLE_FEATURE_X_;
    };
    ```
*   **类静态成员变量 (无论是否`const`)**: 使用 **全大写下划线命名法 (SCREAMING_SNAKE_CASE)**。
    ```cpp
    class Counter
    {
    private:
        static int INSTANCE_COUNT_;
    public:
        static const int MAX_INSTANCES_ALLOWED_ = 100;
    };
    // In .cpp file: int Counter::INSTANCE_COUNT_ = 0;
    ```

### 3.8 宏定义

*   使用 **全大写下划线命名法 (SCREAMING_SNAKE_CASE)**。
*   **谨慎使用宏定义**，优先使用 `const`, `constexpr`, `enum class` 或内联函数。
    ```cpp
    #define MAX_ITERATIONS 100
    ```

## 4. 注释

### 4.1 通用原则

*   **注释应当解释“为什么”而不是“干什么”**。代码本身应该清晰地表达它在做什么。
*   保持注释的更新。
*   **可以使用中文或英文编写注释。**
*   避免过多或不必要的注释。

### 4.2 类与结构体注释

在类或结构体定义之前使用Doxygen风格的注释块，描述其用途。
```cpp
/**
 * @brief 代表一个数据点，包含二维坐标和ID。
 * @brief This class represents a data point with 2D coordinates and an ID.
 * 用于存储和管理点信息。
 * Used to store and manage point information.
 */
class PointData
{
public:
    // ...
};