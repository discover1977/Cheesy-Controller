# Cheesy-Controller
Контроллер двигателя сыроварки.

Контроллер предназначен для управления двигателем аппарата для изготовления сыра.

Функции контроллера:
Контроллер управляет двигателем согласно программе реализованной в прошивке.
Каждая программа может состоять из нескольких циклов.
В рамках одного цикла можно задавать время цикла и режим работы двигателя:
  - двигатель отключен
  - вращение в одну сторону
  - вращение с реверсом (задается время вращения в одну сторону)
  - пауза между переключением направления вращения
  - задание скорости вращения(ШИМ)

Программы:
1. Автоматическая программа сыр Гауда (программы будут добавлятся)
	В автоматической программе возможно изменение скорости вращения двигателя(ШИМ).
	Значение скорости сохраняется в энергонезависимой памяти (EEPROM).
2. Тестовая программа. 
3. Два ручных режима.
  1. Вращение в одну сторону с возможностью изменения скорости (ШИМ).
  1. Вращение с реверсом (по 30 секунд в одну сторону) с возможностью изменения скорости (ШИМ).
