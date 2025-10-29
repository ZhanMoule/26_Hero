# 蜂鸣器模块使用说明

## 模块简介
bsp_buzzer 模块提供了对蜂鸣器硬件的控制功能，基于 STM32 的定时器 PWM 输出实现。该模块不仅支持基础的蜂鸣器开关控制，还实现了完整的音符播放和音乐演奏功能，可用于各种声音提示和音乐播放场景。

## 模块架构
该模块使用对象化的设计方式，通过结构体 `BuzzerInstance_s` 管理蜂鸣器实例，支持多蜂鸣器控制。模块内部封装了定时器配置、PWM控制等底层细节，提供简洁易用的API接口。

## API 功能列表

### 基础函数

| 函数名 | 描述 | 参数 | 返回值 |
| ----- | ---- | ---- | ----- |
| `buzzer_register(Buzzer_Init_Config_s *config)` | 注册蜂鸣器实例 | config: 蜂鸣器配置结构体 | BuzzerInstance_s*: 蜂鸣器实例指针 |
| `buzzer_init(BuzzerInstance_s *buzzer)` | 初始化蜂鸣器 | buzzer: 蜂鸣器实例 | 无 |
| `buzzer_off(BuzzerInstance_s *buzzer)` | 关闭蜂鸣器 | buzzer: 蜂鸣器实例 | 无 |

### 音乐播放函数

| 函数名 | 描述 | 参数 | 返回值 |
| ----- | ---- | ---- | ----- |
| `buzzer_play_note(BuzzerInstance_s *buzzer, uint16_t note, uint16_t octave, float time, uint16_t vol)` | 播放单个音符 | buzzer: 蜂鸣器实例<br>note: 音符(1-7，0为休止符)<br>octave: 八度(0-2)<br>time: 持续拍子<br>vol: 音量 | 无 |
| `buzzer_play_sheet(BuzzerInstance_s *buzzer, float (*sheet)[3], uint16_t len, uint16_t vol)` | 播放音符序列 | buzzer: 蜂鸣器实例<br>sheet: 音符数组[音符,八度,节拍]<br>len: 序列长度<br>vol: 音量 | 无 |
| `buzzer_play_frequency(BuzzerInstance_s *buzzer, uint32_t frequency, float time, uint16_t vol)` | 按指定频率发声 | buzzer: 蜂鸣器实例<br>frequency: 频率值(Hz)<br>time: 持续时间(拍子)<br>vol: 音量 | 无 |

## 配置结构体

| 结构体名 | 描述 | 成员 |
| ----- | ---- | ---- |
| `Buzzer_Init_Config_s` | 蜂鸣器初始化配置 | htim: 定时器句柄<br>channel: PWM通道 |

## 使用说明

### 初始化与配置

在使用蜂鸣器之前，必须先创建配置结构体，注册蜂鸣器实例并进行初始化：

```c
// 声明蜂鸣器实例指针
BuzzerInstance_s *my_buzzer;

// 创建并初始化配置结构体
Buzzer_Init_Config_s buzzer_config;
buzzer_config.htim = &htim4;
buzzer_config.channel = TIM_CHANNEL_1;

// 第1步：注册蜂鸣器实例
my_buzzer = buzzer_register(&buzzer_config);
if (my_buzzer == NULL) {
    // 处理注册失败的情况
    Error_Handler();
}

// 第2步：初始化蜂鸣器
buzzer_init(my_buzzer);
```

### 基本控制

```c
// 关闭蜂鸣器
buzzer_off(my_buzzer);
```

### 播放单个音符

```c
// 播放中音5（音符5，八度1），持续0.5拍，音量1000
buzzer_play_note(my_buzzer, 5, 1, 0.5, 1000);

// 播放高音1（音符1，八度2），持续1拍，音量1000
buzzer_play_note(my_buzzer, 1, 2, 1, 1000);

// 播放低音3（音符3，八度0），持续0.5拍，音量800
buzzer_play_note(my_buzzer, 3, 0, 0.5, 800);

// 播放休止符，持续0.5拍
buzzer_play_note(my_buzzer, 0, 0, 0.5, 0);
```

### 按频率发声

```c
// 按指定频率发声，例如1kHz，持续1拍，音量1000
buzzer_play_frequency(my_buzzer, 1000, 1, 1000);

// 按指定频率发声，例如500Hz，持续0.5拍，音量800
buzzer_play_frequency(my_buzzer, 500, 0.5, 800);
```

### 播放音乐序列

可以自定义音符序列并播放：

```c
// 定义一个简单的音乐序列：小星星前半部分
// 格式：[音符(1-7,0为休止符), 八度(0-2), 节拍值]
float little_star[][3] = {
    {1, 1, 0.5},  // 音符1，八度1，持续0.5拍
    {1, 1, 0.5},  // 音符1，八度1，持续0.5拍
    {5, 1, 0.5},  // 音符5，八度1，持续0.5拍
    {5, 1, 0.5},  // 音符5，八度1，持续0.5拍
    {6, 1, 0.5},  // 音符6，八度1，持续0.5拍
    {6, 1, 0.5},  // 音符6，八度1，持续0.5拍
    {5, 1, 1.0}, // 音符5，八度1，持续1.0拍
    {0, 0, 0.2},  // 休止符，持续0.2拍
    {4, 1, 0.5},  // 音符4，八度1，持续0.5拍
    {4, 1, 0.5},  // 音符4，八度1，持续0.5拍
};

// 播放这个序列
buzzer_play_sheet(my_buzzer, little_star, sizeof(little_star) / sizeof(little_star[0]), 1000);
```

## 音符与频率对照表

模块内部使用以下频率对照表将音符转换为频率（单位：Hz）：

| 音符 | 低音(octave=0) | 中音(octave=1) | 高音(octave=2) |
| ---- | ------------- | ------------- | ------------- |
| 1(Do) | 261.63 | 523.26 | 1046.52 |
| 2(Re) | 293.66 | 587.32 | 1174.64 |
| 3(Mi) | 329.63 | 659.26 | 1318.52 |
| 4(Fa) | 349.23 | 698.46 | 1396.92 |
| 5(Sol) | 392.00 | 784.00 | 1568.00 |
| 6(La) | 440.00 | 880.00 | 1760.00 |
| 7(Si) | 493.88 | 987.76 | 1975.52 |

## 常见问题与解决方案

1. **问题**：蜂鸣器不发声
   **解决**：检查硬件连接、定时器配置是否正确，确保音量值不为0

2. **问题**：音符频率不准确
   **解决**：确认`TIMER_CLOCK_FREQ`宏定义值与实际系统时钟频率一致

3. **问题**：播放速度不符合预期
   **解决**：调整`BEAT_DELAY_TIME`宏定义值，默认为500ms/拍

## 性能与资源占用

- 内存占用：每个蜂鸣器实例消耗约8字节RAM
- 定时器资源：占用一个定时器及其一个PWM通道
- CPU占用：播放音乐时会阻塞CPU执行

## 注意事项

1. **⚠️ 关键步骤**：必须先调用 `buzzer_register()` 注册蜂鸣器实例，然后才能调用 `buzzer_init()` 初始化蜂鸣器
2. 播放音乐会阻塞程序执行，如需非阻塞播放，建议在单独线程中调用
3. `BEAT_DELAY_TIME` 定义了一拍的持续时间(默认500ms)，可根据需要修改
4. 使用 `buzzer_register()` 后检查返回值，如为NULL则表示内存分配失败
5. **⚠️ 重要提示**：`BEAT_DELAY_TIME` 和节拍值 `time` 相乘的结果必须为整数，因为底层使用 HAL_Delay 函数，它只接受整数参数。请确保设置的节拍值能与 BEAT_DELAY_TIME 相乘得到整数毫秒数，否则会因精度损失导致节奏不准确。
6. **⚠️ 重要提示**：使用蜂鸣器模块时，必须先调用 `buzzer_register()` 函数注册蜂鸣器实例，然后才能调用 `buzzer_init()` 函数进行初始化。不遵循此顺序可能导致模块无法正常工作！

## 预置乐谱的使用方法

为了方便使用，模块提供了预置的音乐乐谱集合（在 `music_sheet.h` 和 `music_sheet.c` 文件中）。使用这些预置乐谱可以快速实现音乐播放功能，无需手动定义音符序列。

### 预置乐谱列表

目前包含以下预置乐谱：

1. **新闻联播主题曲**：`news_theme` 数组
2. **猪猪侠主题曲**：`pig_hero_theme` 数组
3. **两只老虎**：`two_tigers` 数组
4. **小星星**：`twinkle_star` 数组
5. **生日快乐歌**：`happy_birthday` 数组
6. **欢乐颂**：`ode_to_joy` 数组
7. **茉莉花**：`jasmine_flower` 数组
8. **爱的罗曼史**：`romance_de_amor` 数组

### 使用方法

首先需要包含乐谱头文件：

```c
#include "music_sheet.h"
```

然后使用 `buzzer_play_sheet()` 函数播放预置乐谱：

```c
// 播放新闻联播主题曲
buzzer_play_sheet(my_buzzer, news_theme, news_theme_len, 1000);

// 播放猪猪侠主题曲
buzzer_play_sheet(my_buzzer, pig_hero_theme, pig_hero_theme_len, 1000);
```

其中：
- `my_buzzer` 是已注册并初始化的蜂鸣器实例
- `news_theme` 和 `pig_hero_theme` 是预定义的乐谱数组
- `news_theme_len` 和 `pig_hero_theme_len` 是对应乐谱的长度（音符数量）
- `1000` 是音量大小

### 在自己的代码中使用预置乐谱

完整的示例代码如下：

```c
#include "bsp_buzzer.h"
#include "music_sheet.h"

void example_function(void) {
    // 声明蜂鸣器实例指针
    BuzzerInstance_s *my_buzzer;
    
    // 创建并初始化配置结构体
    Buzzer_Init_Config_s buzzer_config;
    buzzer_config.htim = &htim4;
    buzzer_config.channel = TIM_CHANNEL_1;
    
    // 第1步：注册蜂鸣器实例
    my_buzzer = buzzer_register(&buzzer_config);
    if (my_buzzer == NULL) {
        // 处理注册失败的情况
        Error_Handler();
    }
    
    // 第2步：初始化蜂鸣器
    buzzer_init(my_buzzer);
    
    // 播放新闻联播主题曲
    buzzer_play_sheet(my_buzzer, news_theme, news_theme_len, 1000);
    
    // 延时一段时间
    HAL_Delay(2000);
    
    // 播放猪猪侠主题曲
    buzzer_play_sheet(my_buzzer, pig_hero_theme, pig_hero_theme_len, 1000);
}
```

### 添加自定义乐谱到乐谱集合

如果需要添加自己的乐谱到乐谱集合中，可以按照以下步骤修改 `music_sheet.h` 和 `music_sheet.c` 文件：

1. 在 `music_sheet.h` 中添加新乐谱的外部声明：

```c
/* 自定义乐谱 */
extern uint16_t my_custom_song[][3];
extern uint16_t my_custom_song_len;
```

2. 在 `music_sheet.c` 中添加新乐谱的定义：

```c
/* 自定义乐谱 */
uint16_t my_custom_song[][3] = {
    {1, 1, 5},  // 音符1，八度1，持续0.5拍
    {2, 1, 5},  // 音符2，八度1，持续0.5拍
    // ... 更多音符
};

/* 自定义乐谱长度 */
uint16_t my_custom_song_len = sizeof(my_custom_song) / sizeof(my_custom_song[0]);
```

完成上述步骤后，就可以像使用预置乐谱一样使用自定义乐谱了。

