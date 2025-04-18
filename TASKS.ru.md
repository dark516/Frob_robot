# Практические задания по робототехнике

## 🛠️ Начальный уровень (Arduino + основы автономности)  
**Цель модуля:** Освоить базовые навыки управления роботом: от индикации статуса до точных перемещений.  
*Актуальность:* Эти технологии лежат в основе сервисных роботов (доставка еды, уборка помещений) и образовательных конструкторов.

---

### 1. «Умная индикация: светофор + кнопка»  
**Современный кейс:** Системы оповещения роботов-курьеров Yandex Rover.  
**Задание:**  
- Подключите 3 светодиода (🔴🟡🟢) и кнопку.  
- Реализуйте:  
  - Последовательное, циклическое мигание всех светодиодов.  
  - Режим «Авария» (все светодиоды мигают синхронно) при долгом удержании.

---

### 2. «Точка А → точка Б: основы движения»  
**Современный кейс:** Траекторное движение роботов-упаковщиков на складах Wildberries.  
**Задание:**  
- Запрограммируйте робота:  
  - Прямолинейное движение на 1 метр вперед.  
  - Разворот на 180° с возвратом в исходную точку.  
  - Визуальная обратная связь через светодиоды (зеленый — движение, красный — остановка).

---

### 3. «Следопыт: линии + звуковая навигация»  
**Современный кейс:** Роботы-сортировщики на конвейерных линиях Tesla.  
**Задание:**  
- Настройте ПИД-регулятор для сложной трассы с перекрестками.  
- Добавьте:  
  - 🎵 Звуковое оповещение при обнаружении препятствия.  
  - 📊 Световую индикацию для разных состояний робота (перемещение по линии 🟢, обнаружение перекрестка 🟡, обнаружение препятствия 🔴)

---

### 4. «Точность — вежливость роботов»  
**Современный кейс:** Позиционирование роботов-хирургов Medtronic.  
**Задание:**  
- Реализуйте:  
  - Управление моторами в м/с через энкодеры.  
  - Поворот на заданный угол с погрешностью ≤ 3°.  
  - Движение на заданные расстояния.
  - Определение пройденного расстояния

---

### Выпускной проект: «Робот-Исследователь»  
Выполните задание, используя полученные навыки

**Задание:**  
- Робот движется по черной линии  
- Останавливается перед препятствиями (используя УЗ-датчик)
- Перед препятствием останавливается на 2 секунды, светит красным светодиодом.
- Съезжая с линии объезжает препятствие по энкодерам и возвращается на маршрут.  


---

## 🤖 Средний уровень (Raspberry Pi + ИИ-зрение)  
**Цель модуля:** Научить робота «видеть» окружение и принимать решения.  
*Актуальность:* Эти навыки используются в системах умного города, сельскохозяйственных дронах и роботах-курьерах.

---

### 1. «Linux для роботов: терминал + автоматизация»  
**Современный кейс:** Управление роем дронов через CLI (стартапы типа Cleo Robotics).  
**Задание:**  
- Познакомьтесь с основами командной строки Linux
- Освойте основные команды перехода между директориями, просмотр файлов и изменение их...

---

### 2. «Компьютерное зрение: камера + OpenCV»  
**Современный кейс:** Детекция дефектов на производственных линиях Siemens.  
**Задание:**  
- Реализуйте:  
  - Трансляцию видео с камеры на ПК.  
  - Распознавание цветовых зон (красный/синий).  

---

### 3. «AR-навигация: маркеры + дополненная реальность»  
**Современный кейс:** Навигация роботов-складчиков по QR-меткам (Amazon Robotics).  
**Задание:**  
- Настройте:  
  - Распознавание ArUco-маркеров.  
  - Наложение AR-текста на видео (координаты маркера).  
- Реализуйте перемещение робота по расположенным на полу ArUco маркерам.

---

### 4. «Слежение: YOLO + трекинг»  
**Современный кейс:** Избегание пешеходов в роботах-доставщиках Serve Robotics.  
**Задание:**  
- Обучите модель YOLO для:  
  - Детекции руки человека.   
  - Поворот робота в сторону движения руки: режим "слежения".

---

#### Выпускной проект: «Автономная машина с ИИ-зрением»  
**Реальный аналог:** Прототипы беспилотных автомобилей Mobileye.  
**Задание:**  
- Робот должен:  
  - Двигаться по полю с разметкой (линии/стены).  
  - Распознавать дорожные знаки через камеру.  
  - Транслировать данные на ПК с визуализацией.  
  - Осуществлять повороты и остановки по отсканированным знакам.  

---

## 🧠 Продвинутый уровень (ROS2 + автономность)  
**Цель модуля:** Научить робота работать в динамических средах как самостоятельный агент.  
*Актуальность:* Эти технологии используются в промышленных роботах (KUKA), космических миссиях (NASA) и умных фабриках.

---

### 1. «ROS2: первая нода + калибровка»  
**Современный кейс:** Управление роботами-погрузчиками на заводах Toyota.  
**Задание:**  
- Познакомьтесь с структурой команд ros2.  
- Напишите вашу первую ноду, отправляющую раз в некоторое время (по умолчанию 1 секунда) сообщение (по умолчанию Hello, ROS2).
  - Добавьте возможность задавания этих параметров через config файл.
  - Добавьте в ваш пакет launch файл запуска этой ноды 

---

### 2. «Одометрия: определение перемещений»  
**Современный кейс:** Определение позиции робота роботы компании Pudu Robotics.  
**Задание:**   
- Настройте отправку данных энкодеров моторов.
- Настройте и откалибруйте пакет модуля гироскопа
- Напишите пакет одометрии, определяя перемещения робота в пространстве по данным датчиков  
- Запустите визуализацию перемещений робота в пространстве

### 2. «SLAM-картограф: лидар + алгоритмы»  
**Современный кейс:** Построение карт складов роботами Amazon.  
**Задание:**   
- Настройте пакет построения карты.
  - Напишите конфигурационный файл slam_toolbox
  - Напишите launch файл запуска slam   
- Реализуйте:
  - Построение карты в ручном режиме.  
  - Визуализацию построеника карты.

---

### 3. «Автономная навигация: цели + препятствия»  
**Современный кейс:** Автономные роботы-уборщики в аэропортах (SoftBank Robotics).  
**Задание:**  
- Настройте пакет навигации робота на базе стека Nav2.
  - Напишите конфигурационный файл nav2
  - Напишите launch файл запуска навигации   
- Реализуйте:
  - Перемещение между точками через RViz.  
  - Объезд статических и динамических препятствий.

---

#### Выпускной проект: «Автономный инспектор объекта»  
**Реальный аналог:** Роботы-инспекторы нефтепроводов Gazprom.  
**Задание:**  
- Робот должен:  
  - Построить карту помещения через SLAM.  
  - Детектировать аномалии на небольших барометрах.  
  - Формировать отчеты.  
  - Вернуться на стартовую точку.  

---
