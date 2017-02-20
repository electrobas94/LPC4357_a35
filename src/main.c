#include "LPC43xx.h"
#include "lpc43xx_i2c.h"
#include "lpc4350_db1.h"
#include "lpc43xx_gpio.h"
#include "lpc43xx_scu.h"
#include "lpc43xx_cgu.h"
#include "lpc43xx_evrt.h"
#include "debug_frmwrk.h"
#include "lpc43xx_uart.h"
#include "m_epos_device.h"
#include "lpc43xx_adc.h"
#include "lpc43xx_gpdma.h"
#include "cmd_adunok.h"
#include "math.h"

#define max_actual_speed 12380

#define kof_mul 132//175*15*152.58 //400522,5  / 3600000 = 0,11125625
#define kof_del 20000//3600*1000     //3600000

// Влияют на точность перемещений и скорость установки позиций
#define max_range 10000 //Растояния начала действия кривой тормажения 
#define min_speed 20 //Минимальный парог скорости при торможении

#define RX_BUF_SIZE 40//Размер буфера для приемма данных с гироскопа
//#define RX_GYRO_DATA 40//Размер полезных данных гироскопа

//PID-controller
#define P1
#define  D1
#define  I20

// Коффициенты преобразования к градусам позиций
#define point_in_deg_h 55543
#define point_in_deg_v (3990/80)
#define full_count_x 199955

volatile int64_t pos_h_cur_stab      = 0;// текущее положение используетсяв двух функциях поэтому глобально
volatile int64_t pos_v_cur_stab      = 0;

volatile uint8_t rx_buf[RX_BUF_SIZE];//Буфер приема данных с гироскопа посредствам DMA

volatile uint8_t Channel1_TC;
GPDMA_Channel_CFG_Type GPDMACfg;

//ГЛОАБАЛЬНЫЕ ПЕРЕМЕННЫЕ СТАБИЛИЗАЦИИ
volatile uint16_t gyro_time=0;//Хранит время между итерациями стабилизации
volatile uint16_t gyro_time_2= 0;// время между итерациями стабилизации
volatile int64_t prev_error_y=0x0;//Предыдущие ошибки по 3-м осям
volatile int64_t prev_error_z=0x0;
volatile int64_t prev_error_x=0x0;

volatile int64_t activ_speed_for_gyro_y = 0;
volatile int64_t activ_speed_for_gyro_z = 0;
volatile int64_t activ_speed_for_gyro_x = 0;

volatile int32_t speed_y_actual_gyro = 0x0;
volatile int32_t speed_x_actual_gyro = 0x0;
volatile int32_t speed_z_actual_gyro = 0x0;

volatile int64_t pos_y_stab = 0;
//volatile int64_t pos_z_stab = 0;
volatile int64_t pos_x_stab = 0;

volatile int64_t prev_speed_giro_y = 0;
volatile int64_t prev_speed_giro_z = 0;
volatile int64_t prev_speed_giro_x = 0;

volatile int64_t start_pos_giro = 0;

//Лимиты по перемещению
volatile int32_t up_limit			=	0;
volatile int32_t down_limit		=	0;
volatile int32_t left_limit		=	0;
volatile int32_t right_limit	=	0;

volatile uint8_t up_limit_f = 0;
volatile uint8_t down_limit_f = 0;
volatile uint8_t left_limit_f = 0;
volatile uint8_t right_limit_f = 0;

// Переменные обслуживания прерывания от БУ_УП
volatile uint8_t leng= 0;
volatile int8_t subs=-1;
volatile uint8_t chs_cmd=0;

// Параметры таймеров
volatile uint32_t timer=0;//Основной таймер отсчета задержек
volatile uint32_t timer_shut_down=10000;//Таймер полного выключения
volatile uint8_t wach_dog_flag=0;//Вкл.Выкл таймера отключения
volatile uint16_t fire_time=0;//Таймер длины очереди
volatile uint8_t stop_flag=0;
// Хранят положение нуля 
volatile int32_t null_pos= 0;
volatile int32_t null_vert= 0;

// Переменные обсалютного и отнасительного смещения по координатам
//Перемещение происходит или нет(флаги)
volatile uint8_t x_target_searth=0;
volatile uint8_t y_target_searth=0;
//Значение позиции цели
volatile int32_t x_target_pos=0;
volatile int32_t y_target_pos=0;
//Преустановленная скорость перемещения
volatile int32_t x_pre_speed=1000;
volatile int32_t y_pre_speed=1000;
// Флаги направление перемещения
volatile uint8_t x_forward_flag=0;
volatile uint8_t y_forward_flag=0;
// Флаг поиска нуля
volatile uint8_t homing_flag=0;

//Текущая скорость перемещения(Без стабилизационной скорости)
volatile int32_t active_y_speed=0;
volatile int32_t active_x_speed=0;
volatile int32_t active_y_pre_speed=0;
volatile int32_t active_x_pre_speed=0;

// Буфер для приема команд от БУ_УП
volatile uint8_t buff_cmd[50];

// Флаг наличия команды в буфере
volatile uint8_t have_cmd=0;

//Флаг стрельбы(очередь или одиночными)
volatile uint8_t fire_flag=0;

// Параметры взвода орудий
volatile uint8_t weapons_time[3]={40, 40, 40};//40 sec на взвод
volatile uint32_t weapons_max_cur[3]={465, 465,465};//Хранит значение в миливольтах(по датчику тока)~5,37 А
volatile uint32_t weapons_max_c_a[3]={5370, 5370,5370};// Хранит не вычисленные значения в милиамперах

// Номер орудия, количество патроноов, номер активной камеры, флаг разрешения стрельбы, флаг включения стабилизации
volatile uint8_t active_weapons=0;
volatile uint16_t shoots_count=0;
volatile uint8_t active_camera=3;
volatile uint8_t fire_on_flag=0;
volatile uint8_t giro_flag=0;

// ФУНКЦИИ

uint8_t get_cmd_length(uint8_t cmd);//Возвращает длину команды по номеру

void move_init_x(void);//Определение направления движения и пре-установка скорости перед относительным или обсалютным смещением
void move_init_y(void);

uint8_t exec_cmd(uint8_t *cmd);//Выполнить команду из буфера

void GINT0_IRQHandler(void);// Прерывания по пролету гильзы
void SysTick_Handler (void);// Системный таймер 1 ms

void find_home_and_init_controller(void);//Поиск нуля и настройка контроллеров

void DMA_IRQHandler (void);// Прерывание по завершению приема пакета данных от гироскопа

int c_entry(void);//Основной цикл пограммы
int main(void);
void sleep(uint32_t tim);

void EVRT_IRQHandler(void);
void calc_chs(uint8_t *cmd,uint8_t len);

void m_board_init(void);// Инициализация компонентов платы

void UART1_IRQHandler(void);//Прерывания по обработке поступающих команд
void UART_IntReceive(void);

void pressue_out(uint8_t *cmd);// Давление. Температура
void temp_out(uint8_t *cmd);

void read_u(uint8_t *cmd);// ток платформы, ток затвора
uint16_t read_cur(void);

void action_shoot(uint8_t *cmd);// Управление затвором
uint8_t action_shoot_forward(void);
uint8_t action_shoot_back(void);

uint8_t read_type_arm(uint8_t *cmd);// Тип вооружения

void set_fier(uint8_t* com);// Стрельба

void stop_move(void);// прекратить движение при отсутствии команд

void dma_gyro_init_sagem(void);

void SetMoveLimits(uint8_t* data);

int main(void)
{
    return c_entry();
}

int c_entry(void)
{
    int32_t result_speed_h = 0;
    int32_t result_speed_v = 0;

    int32_t result_speed_h_pre = 0;
    int32_t result_speed_v_pre = 0;

    //START VAR block
    int32_t correct_speed =0; // use for searth position

    int32_t correct_speed_y = 0;
    int32_t correct_speed_x = 0;

    int64_t pos_y_temp = 0;
    int64_t pos_x_temp = 0;

    // переменный для хранения промежуточных скоростей с гироскопа
    int64_t activ_speed_for_gyro_y_temp = 0;
    int64_t activ_speed_for_gyro_x_temp = 0;

    int64_t speed_y_temp =0;
    int64_t speed_x_temp =0;
    //END VAR block

    homing_flag=1;// Запрет всех действий по перемещению вов время инициализации
    m_board_init();// Выполняем инициализацию компонентов платы
    active_weapons=read_type_arm(buff_cmd);
    
    timer=1500;// ждем 2 секунды включения контроллеров принимая команды
    while(timer!=0)
        if(have_cmd)
            exec_cmd(buff_cmd);

    find_home_and_init_controller();// Включаем контроллеры, настраиваем  ищем ноль
   // dma_gyro_init_sagem();//Включаем дма-контролллер на постоянный прием данных

    y_target_pos=null_vert;
		move_init_y();
				
    x_target_pos=null_pos;
		move_init_x();

    homing_flag=0;
    wach_dog_flag =1;


    while(1)// Основной цикл
    {	
        if(stop_flag)//сбрасывается при приходе любого байто по RS
        {
            stop_move();
            continue;//
        }

        // Для уменьшения задержек должно идти перед основной логикой
        if(have_cmd)
        {
            wach_dog_flag =0;
            exec_cmd(buff_cmd);
            wach_dog_flag =1;
        }

        //Позиция используется в прерывании ДМА, переп\мещение по позиции, лимитах и тд(ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ!!!)
        pos_h_cur_stab = epos_get_actual_position(EPOS_HORIZONTAL);
        
        if(x_target_searth)	// Абсалютное или относительное смещение включено 
        {
            if(x_forward_flag)	// В зависимости от направления развные скорости и условия остановки
            {
                correct_speed	=	x_target_pos-pos_h_cur_stab;
                
                if(	correct_speed	>=	0)	// Цель достигнута
                {
                    active_x_speed=0;
                    epos_velprof_halt(EPOS_HORIZONTAL);
                    x_target_searth=0;
                }
                else
                {
                    if(max_range>correct_speed*(-1))	// Включаем П-регулятор при необходимости	(Для торможения)
                    {
                        active_x_speed	=	(x_pre_speed	*	correct_speed)/max_range;
                        if(active_x_speed*(-1)<min_speed)
                            active_x_speed	=-min_speed;
                    }
                    else
                        active_x_speed	=	x_pre_speed*(-1);
                }
            }
            
            else
            {
                correct_speed	=	pos_h_cur_stab-x_target_pos;
                
                if(correct_speed	>=	0 )
                {
                    epos_velprof_halt(EPOS_HORIZONTAL);
                    active_x_speed=0;
                    x_target_searth=0;
                }
                else
                {
                    if(max_range>correct_speed*(-1))
                    {
                        active_x_speed	=	(x_pre_speed	*	correct_speed)/max_range;
                        active_x_speed*=(-1);
                        if(active_x_speed	<min_speed)
                            active_x_speed	=min_speed;
                    }
                    else
                        active_x_speed	=	x_pre_speed;
                }
            }
        }
        
				pos_v_cur_stab = epos_get_actual_position(EPOS_VERTICAL);
        if(y_target_searth)
        {
            if(y_forward_flag)
            {
                correct_speed	=	y_target_pos-pos_v_cur_stab;
                if(correct_speed	>=	0)
                {
                    active_y_speed=0;
                    epos_velprof_halt(EPOS_VERTICAL);
                    y_target_searth=0;
                }
                else
                {
                    active_y_speed	=	(y_pre_speed	*	correct_speed)/20000;
                    if(active_y_speed*(-1)	>	y_pre_speed)
                        active_y_speed	=	y_pre_speed*(-1);
                    
                    if(active_y_speed*(-1)<20)
                        active_y_speed	=-20;
                }
            }
            else
            {
                correct_speed	=	pos_v_cur_stab-y_target_pos;
                if(correct_speed	>=	0 )
                {
                    active_y_speed=0;
                    epos_velprof_halt(EPOS_VERTICAL);
                    y_target_searth=0;
                }
                else
                {
                    active_y_speed	=	(y_pre_speed	*	correct_speed)/20000;
                    active_y_speed*=(-1);
                    if(active_y_speed	>	y_pre_speed)
                        active_y_speed	=	y_pre_speed;
                    if(active_y_speed	<40)
                        active_y_speed	=40;
                }
            }
        }

        if(giro_flag)
        {
					  //гиро_дата-градусы-поинты+доп дата
            //Вертикальная ось с учетом вектора направления(Z добавлен)
            pos_y_temp = pos_y_stab;
            pos_y_temp *= kof_mul;
            pos_y_temp /= kof_del;

            //Ось горизонтали
            pos_x_temp = pos_x_stab;
            pos_x_temp *= kof_mul;
            pos_x_temp /= kof_del;

            //Сохраняем текущие скорости чтобы они не были изменены в процессе расчетов
            activ_speed_for_gyro_y_temp = activ_speed_for_gyro_y;
            activ_speed_for_gyro_x_temp = activ_speed_for_gyro_x;

            //Кооректировка скорости
            activ_speed_for_gyro_y_temp *= 363;//36
            activ_speed_for_gyro_y_temp /= 1000;

            activ_speed_for_gyro_x_temp *= 363;//36
            activ_speed_for_gyro_x_temp /= 1000;


            //Доп опциия пд регулятара по позиции с отсечкой перелета
            //Выбор горизонтального направления движения
            if(pos_h_cur_stab < pos_x_temp)
            {
                correct_speed_x = pos_x_temp - pos_h_cur_stab;
                speed_x_temp = 1;
            }
            else
            {
                correct_speed_x = pos_h_cur_stab - pos_x_temp;
                speed_x_temp = -1;
            }

            //Выбор вертикального направления движения
            if(pos_v_cur_stab < pos_y_temp)
            {
                correct_speed_y = pos_y_temp - pos_v_cur_stab;
                speed_y_temp = 1;
            }
            else
            {
                correct_speed_y = pos_v_cur_stab - pos_y_temp;
                speed_y_temp = -1;
            }

            //Второй ПД регулятор(позиция)
            if (active_y_speed==0 && !(y_target_searth) )
                speed_y_temp*=(correct_speed_y*30 + (9*correct_speed_y)/gyro_time_2 ) /100; // добавочный регулятор (дополнение и сглаживание)
            else
            {
                activ_speed_for_gyro_y_temp*=113;
                activ_speed_for_gyro_y_temp/=100;
								activ_speed_for_gyro_y_temp+=active_y_speed;

                pos_y_stab = pos_v_cur_stab;
                pos_y_stab *= kof_del;
                pos_y_stab /= kof_mul;
            }

            if(active_x_speed==0 && !(x_target_searth) )
                speed_x_temp*=(correct_speed_x*30 + (9*correct_speed_x)/gyro_time_2 ) /100;
            else
            {
                activ_speed_for_gyro_x_temp*=113;
                activ_speed_for_gyro_x_temp/=100;
								activ_speed_for_gyro_x_temp+=active_x_speed;

                pos_x_stab = pos_h_cur_stab;
                pos_x_stab *= kof_del;
                pos_x_stab /= kof_mul;
            }

						result_speed_v = activ_speed_for_gyro_y_temp+speed_y_temp;
            result_speed_h = activ_speed_for_gyro_x_temp+speed_x_temp;
						
						gyro_time_2 = 0;
        }
        else
        {
            result_speed_v = active_y_speed;
            result_speed_h = active_x_speed;
        }


        // ГОРИЗОНТАЛЬНЫЕ ЛИМИТЫ
        if(left_limit_f)
        {
            if(result_speed_h > 0)
            {
                correct_speed = left_limit - pos_h_cur_stab;
                if(correct_speed<0)
                    correct_speed *= (-1);

                if(correct_speed < max_actual_speed)
                    result_speed_h = (result_speed_h * correct_speed )/max_actual_speed;

                if	(left_limit	<=	pos_h_cur_stab)
								{
                    result_speed_h =0;
										active_x_speed=0;
                    x_target_searth=0;
								}
            }
        }

        if(right_limit_f)
        {
            if(result_speed_h < 0)
            {
                correct_speed = right_limit - pos_h_cur_stab;
                if(correct_speed<0)
                    correct_speed *= (-1);

                if(correct_speed < max_actual_speed)
                    result_speed_h = (result_speed_h * correct_speed )/max_actual_speed;

                if	(right_limit	>=	pos_h_cur_stab)
								{
                    result_speed_h =0;
										active_x_speed=0;
                    x_target_searth=0;
								}
            }
        }

        // ВЕРТИКАЛЬНЫЕ ЛИМИТЫ
        if(up_limit_f)
        {
            if(result_speed_v > 0)
            {
                correct_speed = up_limit - pos_v_cur_stab;
                if(correct_speed<0)
                    correct_speed *= (-1);

                if(correct_speed < max_actual_speed)
                    result_speed_v = (result_speed_v * correct_speed )/max_actual_speed;

                if	(up_limit	<=	pos_v_cur_stab)
								{
                    result_speed_v =0;
										active_y_speed=0;
                    y_target_searth=0;
								}
            }
        }

        if(down_limit_f)
        {
            if(result_speed_v < 0)
            {
                correct_speed = down_limit - pos_v_cur_stab;
                if(correct_speed<0)
                    correct_speed *= (-1);

                if(correct_speed < max_actual_speed)
                    result_speed_v = (result_speed_v * correct_speed )/max_actual_speed;

                if	(down_limit	>=	pos_v_cur_stab)
                {
                    result_speed_v =0;
										active_y_speed=0;
                    y_target_searth=0;
								}
            }
        }

        //Результатирующие скорости джойск, позиция, стабилизация, лимиты
        if( result_speed_v != result_speed_v_pre)
        {
            epos_velprof_set_speed(EPOS_VERTICAL,result_speed_v);
            epos_velprof_move(EPOS_VERTICAL);
            result_speed_v_pre = result_speed_v;
        }

        if( result_speed_h != result_speed_h_pre)
        {
            epos_velprof_set_speed(EPOS_HORIZONTAL,result_speed_h);
            epos_velprof_move(EPOS_HORIZONTAL);
            result_speed_h_pre = result_speed_h;
        }
    }// Конец основного цикла
    
    
}

void stop_move(void)
{
    // Stop Fire
    GPIO_ClearValue(5, 1<<20);
    GPIO_ClearValue(5, 1<<21);
    fire_flag=0;

    //Stop move
    giro_flag=0;
    x_target_searth=0;
    y_target_searth=0;

    active_y_speed=0;
    active_x_speed=0;
    active_y_pre_speed=0;
    active_x_pre_speed=0;

    epos_velprof_set_speed(EPOS_VERTICAL,active_y_speed);
    epos_velprof_move(EPOS_VERTICAL);

    epos_velprof_set_speed(EPOS_HORIZONTAL,active_x_speed);
    epos_velprof_move(EPOS_HORIZONTAL);

    have_cmd=0;
}


uint8_t get_cmd_length(uint8_t cmd)
{
    //Возвращает длинну именно блока данных в команде
    switch(cmd)
    {
    case BU_SET_POS_REL_H:
    case BU_SET_POS_REL_V:
    case BU_SET_POS_ABS_H:
    case BU_SET_POS_ABS_V:
        return 4;
    case BU_PRESET_SPEED_H:
    case BU_PRESET_SPEED_V:
    case BU_SET_SPEED_H:
    case BU_SET_SPEED_V:
    case BU_SET_TV:
    case BU_SET_SHOOT_MAX_TIME:
    case BU_SET_SCOR_CART:
    case BU_SET_FIRE_TIME:
        return 2;
    case BU_SET_SHOOT_MAX_CUR:
        return 3;
    case BU_SET_MOVIE_LIMIT:
        return 5;
    case BU_ACTION_SHOOT:
    case BU_SET_CAMERA:
    case BU_READ_SHOOT_MAX_TIME:
    case BU_READ_SHOOT_MAX_CUR:
    case BU_SET_ENABLE_FIER:
    case BU_SET_FIER:
    case BU_SET_GIRO:
        return 1;
    default:
        return 0;
    }
}

void move_init_x(void)
{
    if(epos_get_actual_position(EPOS_HORIZONTAL)== x_target_pos)
        return;
    
    x_target_searth=1;
    
    if(epos_get_actual_position(EPOS_HORIZONTAL)> x_target_pos) // выбираем направление движения
    {
        active_x_speed=x_pre_speed * (-1);
        x_forward_flag=1;
    }
    else
    {
        active_x_speed=x_pre_speed;
        x_forward_flag=0;
    }
}

void move_init_y(void)
{
    if(epos_get_actual_position(EPOS_VERTICAL)==y_target_pos)
        return;
    
    y_target_searth=1;
    
    if(epos_get_actual_position(EPOS_VERTICAL)>y_target_pos)
    {
        active_y_speed=y_pre_speed * (-1);
        y_forward_flag=1;
    }
    else
    {
        active_y_speed=y_pre_speed;
        y_forward_flag=0;
    }
}

uint8_t exec_cmd(uint8_t *cmd)
{
    //    int32_t stab_tmp = 0;
    //    uint8_t chs_giro=0;
    uint8_t buf_bnic[ 8 ];
    uint8_t length=3;// Длина возвращаемой команды(3 min)
    //    uint8_t i=0;
    uint32_t tmp=0;
    int64_t pos_calc=0;

    buf_bnic[0]=0x4A;
    cmd[0]=0xAA;

    switch(cmd[1])
    {
    case BU_READ_PRESS_OUT://BU_READ_PRESS_OUT
        pressue_out(cmd+2);
        length=5;
        break;//1A 55 FF 00

    case BU_READ_TEMP_OUT://BU_READ_TEMP_OUT
        temp_out(cmd+2);
        length=5;
        break;

    case BU_SET_POS_REL_H:                //BU_SET_POS_REL_H
        if(!homing_flag)
        {
            x_target_pos=epos_get_actual_position(EPOS_HORIZONTAL)+ ((*((int32_t*)(cmd+2)))*2100)/36;// Конвентируем из градусов и з отнасительных координат в обсалютные
            move_init_x();
        }
        break;

    case BU_SET_POS_REL_V://BU_SET_POS_REL_V
        if(!homing_flag)
        {
            y_target_pos=epos_get_actual_position(EPOS_VERTICAL)+ (*((int32_t*)(cmd+2)))*point_in_deg_v;
            move_init_y();
        }
        break;

    case BU_SET_POS_ABS_H:////BU_SET_POS_ABS_H
        if(homing_flag)
            break;
        x_target_pos=((*((int32_t*)(cmd+2)))*2100)/36+null_pos;// Конвентируем из градусов с учетом смещения нуля
        move_init_x();
        break;

    case BU_SET_POS_ABS_V://BU_SET_POS_ABS_V
        if(homing_flag)
            break;
        y_target_pos=(*((int32_t*)(cmd+2)))*point_in_deg_v+null_vert;
        move_init_y();
        break;

    case BU_READ_MOVE_FLAG://BU_READ_MOVE_FLAG
        length=4;
        cmd[2]=(y_target_searth<<1)+ x_target_searth;// Отпровляем флаги перемещения
        break;

    case BU_PRESET_SPEED_H://// BU_PRESET_SPEED_H
        x_pre_speed=*((uint16_t*)(cmd+2)) ;
        break;

    case BU_PRESET_SPEED_V://BU_PRESET_SPEED_V
        y_pre_speed=*((uint16_t*)(cmd+2));
        break;

    case BU_SET_SPEED_H:// BU_SET_SPEED_H
    {
        if(homing_flag)
            break;

        active_x_speed=*((int16_t*)(cmd+2));

        break;
    }

    case BU_SET_SPEED_V://// BU_SET_SPEED_V
    {
        if(homing_flag)
            break;

        active_y_speed=*((int16_t*)(cmd+2));

        break;
    }

    case BU_READ_U://BU_READ_U
        //*(cmd+2)=
        read_u(cmd+2);
        length=5;
        break;

    case BU_ACTION_SHOOT: //BU_ACTION_SHOOT
        cmd[10]=cmd[2];
        cmd[2]=cmd[0]+cmd[1];//CHS

        UART_RS485Send((LPC_USARTn_Type*)LPC_UART1, cmd, 3, 1);//Отправляем первый ответ о приёме команды
        
        cmd[2]=cmd[10];
        action_shoot(cmd+2);
        length=4;
        break;

    case BU_READ_TYPE_ARM://BU_READ_TYPE_ARM
        read_type_arm(cmd+2);
        length=4;
        break;

    case BU_SET_FIER://BU_SET_FIER(Начать огонь)
        set_fier(cmd+2);
        length=4;
        break;

    case BU_SET_FIRE_TIME://BU_SET_FIRE_TIME (длина очереди)
        fire_time=*((uint16_t*)(cmd+2));
        length=3;
        break;

    case BU_SET_ENABLE_FIER://BU_SET_ENABLE_FIER (Разрешение стрельбы)
        fire_on_flag=*(cmd+2);
        break;

    case BU_SET_SCOR_CART://BU_SET_SCOR_CART(Кол-во потронов)
        shoots_count=*((uint16_t*)(cmd+2));
        break;

    case BU_READ_SCOR_CART://BU_READ_SCOR_CART
        *((uint16_t*)(cmd+2))=shoots_count;
        length=5;
        break;

    case BU_SET_GIRO://BU_SET_GIRO
    {
        if(homing_flag)
            break;
        
        if(*(cmd+2))
        {
            pos_y_stab = epos_get_actual_position(EPOS_VERTICAL);
            pos_y_stab *= kof_del;
            pos_y_stab /= kof_mul;

            pos_x_stab = epos_get_actual_position(EPOS_HORIZONTAL);
            pos_x_stab*=kof_del;
            pos_x_stab/=kof_mul;
        }
        else// Ставим текущую скорость(не всегда ноль)
        {
            //active_y_speed =0;
            epos_velprof_set_speed(EPOS_HORIZONTAL, active_x_speed);
            epos_velprof_move(EPOS_HORIZONTAL);
            epos_velprof_set_speed(EPOS_VERTICAL, active_y_speed);
            epos_velprof_move(EPOS_VERTICAL);
        }
        giro_flag=*(cmd+2);
        break;
    }

    case BU_SET_SHOOT_MAX_CUR://BU_SET_SHOOT_MAX_CUR
        tmp=*((uint16_t*)(cmd+3));
        weapons_max_c_a[cmd[2]-1]=tmp;
        //Переводим милиамеры в миливольты датчика
        tmp=(tmp*186)/1000;
        weapons_max_cur[cmd[2]-1]=tmp;
        break;

    case BU_READ_SHOOT_MAX_CUR://BU_READ_SHOOT_MAX_CUR
        *((uint16_t*)(cmd+3))=weapons_max_c_a[cmd[2]-1];
        length=6;
        break;

    case BU_SET_SHOOT_MAX_TIME://BU_SET_SHOOT_MAX_TIME(В секундах)
        weapons_time[cmd[2]-1]=cmd[3];
        break;

    case BU_READ_SHOOT_MAX_TIME://BU_READ_SHOOT_MAX_TIME
        cmd[3]=weapons_time[cmd[2]-1];
        length=4;
        break;
    case BU_READ_CAMERA://BU_READ_CAMERA
        cmd[2]=active_camera;
        length=4;
        break;

    case BU_SET_CAMERA://BU_SET_CAMERA
        buf_bnic[3]=0xCF+cmd[2];
        buf_bnic[2]=cmd[2];
        buf_bnic[1]=0x85;
        buf_bnic[0]=0x4A;
        active_camera=cmd[2];
        UART_Send((LPC_USARTn_Type *)LPC_USART3, buf_bnic, 4, NONE_BLOCKING);
        break;

    case BU_READ_RANGE://BU_READ_RANGE
        buf_bnic[2]=0x27;
        buf_bnic[1]=0xDD;
        buf_bnic[0]=0x4A;
        length=5;

        UART_Send((LPC_USARTn_Type *)LPC_USART3, buf_bnic, 3, NONE_BLOCKING);
        cmd[2]= 0;
        cmd[3]= 0;
        UART_Receive((LPC_USARTn_Type *)LPC_USART3, cmd, 5,NONE_BLOCKING);

        cmd[0]=0xAA;
        cmd[1]=BU_READ_RANGE;
        break;

    case BU_OPEN_GATE://BU_OPEN_GATE
        buf_bnic[3]=0xCD;
        buf_bnic[2]=0xFF;
        buf_bnic[1]=0x84;
        buf_bnic[0]=0x4A;
        UART_Send((LPC_USARTn_Type *)LPC_USART3, buf_bnic, 4, NONE_BLOCKING);
        break;

    case BU_CLOSE_GATE://BU_CLOSE_GATE
        buf_bnic[3]=0xCE;
        buf_bnic[2]=00;
        buf_bnic[1]=0x84;
        buf_bnic[0]=0x4A;
        UART_Send((LPC_USARTn_Type *)LPC_USART3, buf_bnic, 4, NONE_BLOCKING);
        break;

#define mul_df (399/8)
    case BU_READ_POS_H://BU_READ_POS_H
        length=7;
        if(!homing_flag)
        {
            pos_calc=(pos_h_cur_stab-null_pos)%2100000;
            pos_calc=(pos_calc*36)/2100;

            if(pos_calc<0)
                pos_calc=36000+pos_calc;

            if(pos_calc==36000)
                pos_calc=0;
            *((int32_t*)(cmd+2))=pos_calc;
        }
        else
            *((int32_t*)(cmd+2))=0;
        break;

    case BU_READ_POS_V://BU_READ_POS_V
        length=7;

        if(!homing_flag)
            *((int32_t*)(cmd+2))=(((pos_v_cur_stab-null_vert)/mul_df)%36000);//%36000);
        else
            *((int32_t*)(cmd+2))=0;
        break;

    case BU_READ_ACT_SPEED_H://BU_READ_ACT_SPEED_H
        length=7;
        if(!homing_flag)
            *((int32_t*)(cmd+2))=epos_get_actual_speed(EPOS_HORIZONTAL);//EDIT THIS LINE!!!!!!!!!!!!!!!!!!!
        else
            *((int32_t*)(cmd+2))=0;
        break;

    case BU_READ_ACT_SPEED_V://BU_READ_ACT_SPEED_V
        length=7;
        if(!homing_flag)
            *((int32_t*)(cmd+2))=epos_get_actual_speed(EPOS_VERTICAL);
        else
            *((int32_t*)(cmd+2))=0;
        break;

    case BU_SET_MOVIE_LIMIT:
    {
        length=4;
        if(homing_flag)
            break;
        SetMoveLimits(cmd+2);
        break;
    }

    default://COMAND NOT FOUND
        return 1;
    }
    
    calc_chs(cmd, length);
    
    have_cmd=0;
    
    UART_RS485Send((LPC_USARTn_Type*)LPC_UART1, cmd, length, 1);// Отправляем ответ

    return 0;
}

void SetMoveLimits(uint8_t* data)
{
#define d_up_limit 0x01
#define d_down_limit 0x02
#define d_left_limit 0x03
#define d_right_limit 0x04
#define d_up_limit_r 0x81
#define d_down_limit_r 0x82
#define d_left_limit_r 0x83
#define d_right_limit_r 0x84

    int32_t lim_val = *(int32_t*)(data+1);

    switch (data[0])
    {
    case d_up_limit:
    {
        lim_val = lim_val*point_in_deg_v+null_vert;
        up_limit = lim_val;
        up_limit_f = 1;
        break;
    }
    case d_down_limit:
    {
        lim_val = lim_val*point_in_deg_v+null_vert;
        down_limit = lim_val;
        down_limit_f = 1;
        break;
    }
    case d_right_limit:
    {
        lim_val*=(-1);
        lim_val = (lim_val*2100)/36+null_pos;
        right_limit = lim_val;
        right_limit_f = 1;
        break;
    }
    case d_left_limit:
    {
        lim_val*=(-1);
        lim_val = (lim_val*2100)/36+null_pos;
        left_limit = lim_val;
        left_limit_f = 1;
        break;
    }
    case d_up_limit_r:
    {
        up_limit_f = 0;
        break;
    }
    case d_down_limit_r:
    {
        down_limit_f = 0;
        break;
    }
    case d_right_limit_r:
    {
        right_limit_f = 0;
        break;
    }
    case d_left_limit_r:
    {
        left_limit_f = 0;
        break;
    }
    }
}

void GINT0_IRQHandler(void)
{
    uint32_t ctrl = LPC_GPIO_GROUP_INT0->CTRL;
    
    // Check if interrupt is pending
    if (ctrl & 1)
    {
        // Clear interrupt by writing 1 to the INT bit 0.
        LPC_GPIO_GROUP_INT0->CTRL = ctrl;
        
        if(shoots_count!=0)
            shoots_count--;
    }
}

void SysTick_Handler (void)// SysTick interrupt 1 ms
{
    if(wach_dog_flag)
    {
        if(timer_shut_down==0)//после 10 секунд посылаем команду на отключение питания
        {
            buff_cmd[0] = 0x1A;
            buff_cmd[1] = 0x55;
            buff_cmd[2] = 0x00;
            buff_cmd[3] = 0x6F;
            UART_RS485Send((LPC_USARTn_Type*)LPC_UART1, buff_cmd,4,1);
            wach_dog_flag = 0;
        }
        else
            if(timer_shut_down==9000)// После одной екунды останавливаем все действия
                stop_flag=1;
            else if(timer_shut_down==10000)
                stop_flag=0;
        timer_shut_down--;
    }
    if(giro_flag)
    {
        gyro_time++;
        gyro_time_2++;
    }
    
    if(timer!=0)
        timer--;
    else
        if(fire_flag)
        {
            GPIO_ClearValue(5, 1<<20);
            GPIO_ClearValue(5, 1<<21);
            fire_flag=0;
        }
}

void find_home_and_init_controller(void)
{
    uint8_t i;

    // Включаем контроллеры
    epos_device_enable(EPOS_HORIZONTAL);
    epos_set_velosity_profile(EPOS_HORIZONTAL);
    epos_device_enable(EPOS_VERTICAL);
    epos_set_velosity_profile(EPOS_VERTICAL);
    
    for(i=0;i<100;i++)// :Опрашиваем статус и ждем включения
    {
        if(!(epos_device_get_status(EPOS_HORIZONTAL) &(1<<1)==(1<<1)))
        {
            epos_device_enable(EPOS_HORIZONTAL) ;
            epos_set_velosity_profile(EPOS_HORIZONTAL);
        }
        if(!(epos_device_get_status(EPOS_VERTICAL) &(1<<1)==(1<<1)))
        {
            epos_device_enable(EPOS_VERTICAL) ;
            epos_set_velosity_profile(EPOS_VERTICAL);
        }
        if(have_cmd)
            exec_cmd(buff_cmd);
    }

    //Ставим максимальные значения ускорения и торможения
    epos_velprof_set_accselerations(EPOS_HORIZONTAL, 0xFFFFFFFF>>1);
    epos_velprof_set_accselerations(EPOS_VERTICAL, 0xFFFFFFFF>>1);
    epos_velprof_set_durations(EPOS_HORIZONTAL, 0xFFFFFFFF>>1);
    epos_velprof_set_durations(EPOS_VERTICAL, 0xFFFFFFFF>>1);

    // Начинаем поиск

    if (!(GPIO_ReadValue(1) & (1<<14) ))// если мы находимся на датчике отезжаем назад
    {
        epos_velprof_set_speed(EPOS_HORIZONTAL, 1000);
        epos_velprof_move(EPOS_HORIZONTAL);
        timer = 1500;
        while(timer!=0)
        {
            epos_get_actual_position(EPOS_HORIZONTAL);
            if(have_cmd)
                exec_cmd(buff_cmd);
        }
    }

    epos_velprof_set_speed(EPOS_HORIZONTAL, -2500);
    epos_velprof_move(EPOS_HORIZONTAL);
    
    while(1)
    {
        if (!(GPIO_ReadValue(1) & (1<<14) ))// Движемся пока не наехали на датчик нуля
        {
            epos_velprof_set_speed(EPOS_HORIZONTAL, 0);
            epos_velprof_move(EPOS_HORIZONTAL);
            null_pos=epos_get_actual_position(EPOS_HORIZONTAL);
            break;
        }
        if(have_cmd)// Отвечаем на некоторые команды
            exec_cmd(buff_cmd);
    }

    epos_velprof_set_speed(EPOS_VERTICAL, -1500);
    epos_velprof_move(EPOS_VERTICAL);
    
    while(1)
    {
        if(!(epos_device_get_status(EPOS_VERTICAL) & (1<<1)==(1<<1)))// Едим пока не наехали на концевик
        {
            break;
        }
        if(have_cmd)
            exec_cmd(buff_cmd);
    }
    
    for(i=0;i<100;i++)// Ждем включения обоих контроллеров
    {
        if(!(epos_device_get_status(EPOS_HORIZONTAL) &(1<<1)==(1<<1)))
        {
            epos_device_enable(EPOS_HORIZONTAL) ;
            epos_set_velosity_profile(EPOS_HORIZONTAL);
        }
        if(!(epos_device_get_status(EPOS_VERTICAL) &(1<<1)==(1<<1)))
        {
            epos_device_enable(EPOS_VERTICAL) ;
            epos_set_velosity_profile(EPOS_VERTICAL);
        }
        if(have_cmd)
            exec_cmd(buff_cmd);
    }
    epos_device_preset_speed(EPOS_VERTICAL,1500);
    epos_device_move_rel(EPOS_VERTICAL,100200);// Едим от концевика в ноль

    null_vert=epos_get_actual_position(EPOS_VERTICAL); // Записываем полученную точку как нулевую
}


void DMA_IRQHandler (void)
{
    uint32_t tmp;
    int64_t speed_giro_y=0;
    int64_t speed_giro_z=0;
    int64_t speed_giro_x=0;
    char tmp_buf[15];
    int64_t y_cor_angle_kof;

    // Scan interrupt pending
    for (tmp = 0; tmp <= 7; tmp++)
    {
        if (LPC_GPDMA->INTSTAT & GPDMA_DMACIntStat_Ch(tmp))
        {
            // Check counter terminal status
            if (LPC_GPDMA->INTTCSTAT &GPDMA_DMACIntTCStat_Ch(tmp))
            {
                // Clear terminate counter Interrupt pending
                LPC_GPDMA->INTTCCLEAR =GPDMA_DMACIntTCClear_Ch(tmp);
                
                switch (tmp){
                case 0:
                    GPDMA_ChannelCmd(0, DISABLE);
                    break;
                case 1://Получили пакет и снова включаем канал
                    Channel1_TC++;
                    GPDMA_Setup(&GPDMACfg);
                    GPDMA_ChannelCmd(1, ENABLE);

                    if(gyro_time==0)
                        break;

                    // Ось Y
                    tmp_buf[1]=rx_buf[9]>>6 | rx_buf[8]<<2;
                    tmp_buf[2]=rx_buf[8]>>6 | rx_buf[7]<<2;
                    tmp_buf[3]=rx_buf[7]>>6;

                    if(tmp_buf[3]& 0x2)
                    {
                        tmp_buf[3]= tmp_buf[3] | 0xFC;
                        tmp_buf[4] = 0xFF;
                    }
                    else
                        tmp_buf[4] = 0x00;

                    // Ось Z
                    tmp_buf[5]=rx_buf[6]>>6 | rx_buf[5]<<2;
                    tmp_buf[6]=rx_buf[5]>>6 | rx_buf[4]<<2;
                    tmp_buf[7]=rx_buf[4]>>6;

                    if(tmp_buf[7]& 0x2)
                    {
                        tmp_buf[7]= tmp_buf[7] | 0xFC;
                        tmp_buf[8] = 0xFF;
                    }
                    else
                        tmp_buf[8] = 0x00;

                    // Ось X
                    tmp_buf[9]=rx_buf[3]>>6 | rx_buf[2]<<2;
                    tmp_buf[10]=rx_buf[2]>>6 | rx_buf[1]<<2;
                    tmp_buf[11]=rx_buf[1]>>6;

                    if(tmp_buf[11]& 0x2)
                    {
                        tmp_buf[11]= tmp_buf[11] | 0xFC;
                        tmp_buf[12] = 0xFF;
                    }
                    else
                        tmp_buf[12] = 0x00;


                    prev_speed_giro_y = activ_speed_for_gyro_y;
                    prev_speed_giro_x = activ_speed_for_gyro_x;

                    speed_giro_y=*((int32_t*)(tmp_buf+1));
                    speed_giro_z=*((int32_t*)(tmp_buf+5));
                    speed_giro_x=*((int32_t*)(tmp_buf+9));

                    speed_giro_x*= (-1);

                    // компенсация ухода Y
                    // Проявляется на малых скоростях больше
                    speed_giro_y+=67; //66 
                    if(speed_giro_y>-90 && speed_giro_y<90)
                        speed_giro_y+=6;

                    // компенсация ухода X
                    speed_giro_x-=16;
                    //if(speed_giro_x>-90 && speed_giro_x<90)
                    //speed_giro_x+=6;

                    // компенсация ухода Z
                    speed_giro_z -=51;
                    if(speed_giro_z>-90 && speed_giro_z<90)
											speed_giro_z -=6;

                    //Учитываем влияние осей

                    //A1. Коректировка влтяния в зависимости от положения
                    y_cor_angle_kof =(pos_h_cur_stab - null_pos);//%2100000;
										
										activ_speed_for_gyro_y = speed_giro_y * cos(y_cor_angle_kof*360.*3.1415926/180/2100000);
										activ_speed_for_gyro_z = (-1)*speed_giro_z * sin(y_cor_angle_kof*360.*3.1415926/180/2100000);

                    activ_speed_for_gyro_y += activ_speed_for_gyro_z;
                    //A1 end.

                    activ_speed_for_gyro_x = speed_giro_x;

                    pos_y_stab += (gyro_time * ((activ_speed_for_gyro_y + prev_speed_giro_y)))
                            + 20 * (activ_speed_for_gyro_y - prev_speed_giro_y) / gyro_time;

                    pos_x_stab += (gyro_time * ((speed_giro_x + prev_speed_giro_x)))
                            + 20 * (speed_giro_x - prev_speed_giro_x) / gyro_time;


                    gyro_time = 0;

                    break;
                default:
                    break;
                }
                
            }
            // Check error terminal status
            if (LPC_GPDMA->INTERRSTAT & GPDMA_DMACIntErrStat_Ch(tmp))
            {
                // Clear error counter Interrupt pending
                LPC_GPDMA->INTERRCLR =GPDMA_DMACIntErrClr_Ch(tmp);
                switch (tmp){
                case 0:
                    GPDMA_ChannelCmd(0,DISABLE);
                    break;
                case 1:
                    GPDMA_Setup(&GPDMACfg);
                    GPDMA_ChannelCmd(1, ENABLE);
                    break;
                default:
                    break;
                }
            }
        }
    }
    
}

void dma_gyro_init_sagem(void)
{
    //Блок для работы с гироскопом Sagem
back_init_giro:
    GPDMACfg.TransferSize= 1;// Включаем DMA
    GPDMA_Setup(&GPDMACfg);
    GPDMA_ChannelCmd(1,ENABLE);

    while(rx_buf[0]!=0x5A);// Ждем пока не придет синхробайт
    Channel1_TC=0;
    while(Channel1_TC != 9);//ждем пока не придет весь пакет данных

    GPDMACfg.TransferSize = 10;//Пернастраеваем ДМА на прием пакетов данных
    GPDMA_Setup(&GPDMACfg);
    GPDMA_ChannelCmd(1, ENABLE);

    Channel1_TC=0;

    while(Channel1_TC < 4);//ждем приема нескольких пакетов(что бы проверить синхронность потока)

    if(rx_buf[0]!= 0x5A)// если не синхронно начинаем заново
        goto back_init_giro;

    Channel1_TC=0;
}

void sleep(uint32_t tim)
{
    timer=tim;
    while(timer!=0);
}

void EVRT_IRQHandler(void)
{
    uint32_t reg;
    reg = LPC_EVENTROUTER->STATUS;
    
    if(reg & (1<<CAN_ERIn)) // CAN_ERIn interrupt flag
    {
        LPC_EVENTROUTER->CLR_STAT = (1<<CAN_ERIn);// clear Eventrouter CAN_ERIn interrupt flag
        CAN_IRQHandler();
    }
}

void calc_chs(uint8_t *cmd, uint8_t len)
{
    uint8_t i;
    uint8_t chs_v=0;
    for(i=0;i<len-1;i++)
        chs_v+=cmd[i];
    
    cmd[len-1]=chs_v;
}

void m_board_init(void)
{
    UART_RS485_CTRLCFG_Type rs485cfg;
    UART_CFG_Type UARTConfigStruct;
    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    
    SystemInit();
    CGU_Init();
    SysTick_Config(CGU_GetPCLKFrequency(CGU_PERIPHERAL_M4CORE) / 1000); // Таймер раз в 1 мс
    
    //Выводы светодиодов на плате
    scu_pinmux(D3_SCU_PORT, D3_SCU_PIN, MD_BUK, FUNC4);
    GPIO_SetDir(D3_GPIO_PORT, D3_GPIO_MASK, 1);
    scu_pinmux(D4_SCU_PORT, D4_SCU_PIN, MD_BUK, FUNC4);
    GPIO_SetDir(D4_GPIO_PORT, D4_GPIO_MASK, 1);
    scu_pinmux(D5_SCU_PORT, D5_SCU_PIN, MD_BUK, FUNC4);
    GPIO_SetDir(D5_GPIO_PORT, D5_GPIO_MASK, 1);
    scu_pinmux(D6_SCU_PORT, D6_SCU_PIN, MD_BUK, FUNC4);
    GPIO_SetDir(D6_GPIO_PORT, D6_GPIO_MASK, 1);
    
    //Выводы кнопок на плате
    scu_pinmux(S1_SCU_PORT, S1_SCU_PIN, MD_BUK | MD_EZI, FUNC4);
    GPIO_SetDir(S1_GPIO_PORT, S1_GPIO_MASK, 0);
    scu_pinmux(S2_SCU_PORT, S2_SCU_PIN, MD_BUK | MD_EZI, FUNC4);
    GPIO_SetDir(S2_GPIO_PORT, S2_GPIO_MASK, 0);
    scu_pinmux(S3_SCU_PORT, S3_SCU_PIN, MD_BUK | MD_EZI, FUNC4);
    GPIO_SetDir(S3_GPIO_PORT, S3_GPIO_MASK, 0);
    scu_pinmux(S4_SCU_PORT, S4_SCU_PIN, MD_BUK | MD_EZI, FUNC4);
    GPIO_SetDir(S4_GPIO_PORT, S4_GPIO_MASK, 0);
    
    //Датчик нуля
    scu_pinmux(0x3, 0x4, MD_BUK | MD_EZI, FUNC0);
    GPIO_SetDir(1, 1<<14, 0);
    
    //CAN1 pins
		scu_pinmux(0x4,8,MD_PLN,FUNC6);/* PE.3 CAN TD1, FUNC1 */
    scu_pinmux(0x4,9,MD_PLN | MD_EZI ,FUNC6);/* PE.2CAN RD1, FUNC1 */

		//CAN0 pins
    scu_pinmux(0x3,2,MD_PLN,FUNC2);/* PE.3 CAN TD1, FUNC1 */
    scu_pinmux(0x3,1,MD_PLN | MD_EZI ,FUNC2);/* PE.2CAN RD1, FUNC1 */
    
    //UASART3 pins БНИЦ Rs 232
    scu_pinmux(0x2 ,3 , MD_PDN, FUNC2);
    scu_pinmux(0x2 ,4 , MD_PLN|MD_EZI|MD_ZI, FUNC2);
    
    UART_ConfigStructInit(&UARTConfigStruct);
    UARTConfigStruct.Baud_rate = 57600;
    UART_Init((LPC_USARTn_Type *)LPC_USART3, &UARTConfigStruct);
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    UART_FIFOConfig((LPC_USARTn_Type *)LPC_USART3, &UARTFIFOConfigStruct);
    UART_TxCmd((LPC_USARTn_Type *)LPC_USART3, ENABLE);
    
    UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig((LPC_USARTn_Type *)LPC_USART3, UART_INTCFG_RLS, ENABLE);
    
    //UART1 - БУ_УП RS-485UART1 - CMD LINE
    scu_pinmux(0xC ,13 , MD_PDN, FUNC2); // PC.13 : UART1_TXD
    scu_pinmux(0xC ,14 , MD_PLN|MD_EZI|MD_ZI, FUNC2); // PC.14 : UART1_RXD
    scu_pinmux(0xC ,12 , MD_PDN, FUNC2); // PC.14 : UART1_RXD
    
    UART_ConfigStructInit(&UARTConfigStruct);
    UARTConfigStruct.Baud_rate = 115200;
    
    UART_Init((LPC_USARTn_Type *)LPC_UART1, &UARTConfigStruct);
    
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    
    UART_FIFOConfig((LPC_USARTn_Type *)LPC_UART1, &UARTFIFOConfigStruct);
    
    rs485cfg.AutoDirCtrl_State = ENABLE;
    rs485cfg.DirCtrlPin = UART_RS485_DIRCTRL_DTR;
    rs485cfg.DirCtrlPol_Level = SET;
    rs485cfg.DelayValue = 50;
    rs485cfg.NormalMultiDropMode_State = DISABLE;
    rs485cfg.AutoAddrDetect_State = ENABLE;
    rs485cfg.MatchAddrValue = 0;
    rs485cfg.Rx_State = ENABLE;
    UART_RS485Config((LPC_USARTn_Type *)LPC_UART1, &rs485cfg);
    
    UART_IntConfig((LPC_USARTn_Type *)LPC_UART1, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig((LPC_USARTn_Type *)LPC_UART1, UART_INTCFG_RLS, ENABLE);
    
    NVIC_SetPriority(UART1_IRQn, ((0x01<<3)|0x01));
    NVIC_EnableIRQ(UART1_IRQn);
    
    // Enable UART Transmit
    UART_TxCmd((LPC_USARTn_Type *)LPC_UART1, ENABLE);
    
    
    GPDMA_Init();// Initialize GPDMA controller
    
    NVIC_DisableIRQ (DMA_IRQn);// Setting GPDMA interrupt
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));

    //UART_0 GIRO RS_485//UART_0 GIRO RS_485
    scu_pinmux(0xF ,10 , MD_PDN, FUNC1); // P2.0 : UART0_TXD
    scu_pinmux(0xF ,11 , (MD_PLN|MD_EZI|MD_ZI), FUNC1); // P2.1 : UART0_RXD
    
    UART_ConfigStructInit(&UARTConfigStruct);
    UARTConfigStruct.Baud_rate = 115200;
    
    UART_Init((LPC_USARTn_Type *)LPC_USART0, &UARTConfigStruct);
    
    UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
    
    UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE;//dma
    UART_FIFOConfig((LPC_USARTn_Type *)LPC_USART0, &UARTFIFOConfigStruct);
    
    UART_IntConfig(LPC_USART0, UART_INTCFG_RBR, ENABLE);
    UART_IntConfig(LPC_USART0, UART_INTCFG_RLS, ENABLE);
    
    // Enable UART Transmit
    UART_TxCmd((LPC_USARTn_Type *)LPC_USART0, ENABLE);
    
    GPDMA_Init();// Initialize GPDMA controller
    
    NVIC_DisableIRQ (DMA_IRQn);// Setting GPDMA interrupt
    /* preemption = 1, sub-priority = 1 */
    NVIC_SetPriority(DMA_IRQn, ((0x01<<3)|0x01));
    
    // Setup GPDMA channel --------------------------------
    // channel 1(RX)
    GPDMACfg.ChannelNum = 1;
    // Source memory - don't care
    GPDMACfg.SrcMemAddr = 0;
    // Destination memory
    GPDMACfg.DstMemAddr = (uint32_t) &rx_buf;
    // Transfer size
    GPDMACfg.TransferSize = sizeof(rx_buf);
    // Transfer width - don't care
    GPDMACfg.TransferWidth = 0;
    // Transfer type
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA;
    // Source connection
    GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;
    // Destination connection - don't care
    GPDMACfg.DstConn = 0;
    // Linker List Item - unused
    GPDMACfg.DMALLI = 0;
    GPDMA_Setup(&GPDMACfg);
    
    /* Reset terminal counter */
    Channel1_TC = 0;

    // Enable interrupt for DMA
    NVIC_EnableIRQ (DMA_IRQn);
    
    //CAN 12 MHz //CAN INIT																													
		CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_BASE_APB3);
		CGU_EntityConnect(CGU_CLKSRC_XTAL_OSC, CGU_BASE_APB1);//CAN_1
    
    //EVENT ROUTER INIT
    EVRT_Init(LPC_EVENTROUTER);
    EVRT_ClrPendIntSrc(LPC_EVENTROUTER, EVRT_SRC_CCAN);
    
    CAN_Init( CAN_BITRATE1000K12MHZ, CLKDIV1 , TX_callback, RX_callback);
    
    // Disable SLEEPDEEP
    SCB->SCR = 0;
    
    //Enable source interrupt router for can
    EVRT_ConfigIntSrcActiveType(LPC_EVENTROUTER, EVRT_SRC_CCAN,
                                
                                EVRT_SRC_ACTIVE_RISING_EDGE);
    EVRT_SetUpIntSrc(LPC_EVENTROUTER, EVRT_SRC_CCAN, ENABLE);
    NVIC_SetPriority(EVENTROUTER_IRQn,0);
    NVIC_EnableIRQ(EVENTROUTER_IRQn);
    
    can_init();
    
    //I2C initcialize
    I2C_Init(LPC_I2C0, 100000);
    I2C_Cmd(LPC_I2C0, ENABLE);
    
    // ADC initialize
    ADC_IntConfig(LPC_ADC0,ADC_ADINTEN0,DISABLE);
    
    //DAC
    scu_pinmux(4, 4, MD_BUK, FUNC0);
    GPIO_SetDir(2, 1<<4 , 1);
    GPIO_SetValue(2, 1<<4 );
    
    //Fire 0
    scu_pinmux(0xB, 0, MD_BUK, 0x04);
    GPIO_SetDir(5, 1<<20 , 1);
    GPIO_ClearValue(5, 1<<20);
    
    //Fire 1
    scu_pinmux(0xB, 1, MD_BUK, 0x04);
    GPIO_SetDir(5, 1<<21 , 1);
    GPIO_ClearValue(5, 1<<21);
    
    //Count shoot
    scu_pinmux(0x6, 0x3, MD_BUK | MD_EZI, FUNC0);
    GPIO_SetDir(3, 1<<2,0);
    
    // Configure interrupt from shoot counter
    LPC_GPIO_GROUP_INT0->CTRL = 0x01;   // clear interrupt, OR, edge
    
    ((uint32_t *)&(LPC_GPIO_GROUP_INT0->PORT_POL0))[3] &= ~(1<<2); // falling edge
    ((uint32_t *)&(LPC_GPIO_GROUP_INT0->PORT_ENA0))[3] |= 1<<2;  // enable interrupt
    
    NVIC_EnableIRQ(GINT0_IRQn);
    
    //Shoot action
    scu_pinmux(0xA, 0x4, MD_BUK, FUNC4);
    GPIO_SetDir(5, 1<<19,1);
    GPIO_ClearValue(5, 1<<19);
    scu_pinmux(0x9, 0x4, MD_BUK, FUNC4);
    GPIO_SetDir(5, 1<<17,1);
    GPIO_ClearValue(5, 1<<17);
    
    //plat ag. Ender action shoot gerenade
    scu_pinmux(0xA, 3, MD_BUK, FUNC0);
    GPIO_SetDir(4, 1<<10 , 1);
    GPIO_ClearValue(4, 1<<10);
    GPIO_SetValue(4, 1<<10);
    
    //Выводы вооружения
    scu_pinmux(0xA, 1, MD_PUP | MD_EZI, FUNC0);
    GPIO_SetDir(4, 1<<8, 0);
    scu_pinmux(0xA, 2, MD_PUP | MD_EZI, FUNC0);
    GPIO_SetDir(4, 1<<9, 0);
}

void UART1_IRQHandler(void)
{
    uint32_t intsrc, tmp, tmp1;
    
    /* Determine the interrupt source */
    intsrc = LPC_UART1->IIR;
    tmp = intsrc & UART_IIR_INTID_MASK;
    
    // Receive Line Status
    if (tmp == UART_IIR_INTID_RLS)
    {
        tmp1 = LPC_UART1->LSR;// Check line status
        // Mask out the Receive Ready and Transmit Holding empty status
        tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
    }
    
    // Receive Data Available or Character time-out
    if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
        UART_IntReceive();
}

void UART_IntReceive(void)
{
    uint8_t tmpc;
    uint32_t rLen;
    
    while(1)
    {
        rLen = UART_Receive((LPC_USARTn_Type *)LPC_UART1, &tmpc, 1, NONE_BLOCKING);
        
        if (rLen)// Данные получены
        {
            if(subs==-1 && tmpc==0x2A)// Начала пакета
            {
                subs=-2;
                timer_shut_down= 10000;
                chs_cmd=0x2A;
            }
            else
            {
                if(subs==-2)// Вычисляем длину
                {
                    leng=2;
                    subs=get_cmd_length(tmpc)+1;
                    buff_cmd[1]=tmpc;
                    chs_cmd+=tmpc;
                }
                else
                {
                    buff_cmd[leng]=tmpc;
                    if(leng==subs+1)
                    {
                        subs=-1;
                        if(chs_cmd==tmpc)
                            have_cmd=1;
                    }
                    else
                    {
                        chs_cmd+=tmpc;
                        leng++;
                    }
                }
            }
        }
        // no more data
        else
            break;
    }
}

void pressue_out(uint8_t *cmd)
{
    I2C_M_SETUP_Type transferMCfg;
    uint8_t ret_data[2];
    
    transferMCfg.sl_addr7bit = 0x78;
    transferMCfg.tx_data = NULL ;
    transferMCfg.tx_length = 0;
    transferMCfg.rx_data = ret_data;
    transferMCfg.rx_length = 2;
    transferMCfg.retransmissions_max = 30;
    
    I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
    
    cmd[0]=ret_data[1];
    cmd[1]=ret_data[0];
}

void temp_out(uint8_t *cmd)
{
    I2C_M_SETUP_Type transferMCfg;
    uint8_t ret_data[2];
    
    ret_data[0]=0x0;
    
    transferMCfg.sl_addr7bit = 72;
    transferMCfg.tx_data = ret_data ;
    transferMCfg.tx_length = 1;
    transferMCfg.rx_data = ret_data;
    transferMCfg.rx_length = 2;
    transferMCfg.retransmissions_max = 30;
    
    I2C_MasterTransferData(LPC_I2C0, &transferMCfg, I2C_TRANSFER_POLLING);
    
    cmd[0]=ret_data[0];
    cmd[1]=ret_data[1];
}

void read_u(uint8_t *cmd)
{
    uint16_t adc_value;
    
    ADC_Init(LPC_ADC0, 100000, 10);
    
    ADC_ChannelCmd(LPC_ADC0,ADC_CHANNEL_3,ENABLE);
    ADC_StartCmd(LPC_ADC0,ADC_START_NOW);
    
    //Wait conversion complete
    while (!(ADC_ChannelGetStatus(LPC_ADC0,ADC_CHANNEL_3,ADC_DATA_DONE)));
    adc_value = ADC_ChannelGetData(LPC_ADC0,ADC_CHANNEL_3);
    
    ADC_ChannelCmd(LPC_ADC0,ADC_CHANNEL_3,DISABLE);
    
    adc_value=adc_value*36;
    *((uint16_t*)cmd)=adc_value;
    
    ADC_DeInit(LPC_ADC0);
    
    //CAN_Init( CAN_BITRATE1000K12MHZ, CLKDIV1 , TX_callback, RX_callback);// КОСТЫЛЬ. так как САN  и ADC0 висит на одной шине он дает збой
}

uint16_t read_cur(void)//zatwor
{
    uint16_t timeout=3000;
    uint16_t adc_value=0;
    
    ADC_Init(LPC_ADC0, 100000, 10);
    
    ADC_ChannelCmd(LPC_ADC0,ADC_CHANNEL_0,ENABLE);
    ADC_StartCmd(LPC_ADC0,ADC_START_NOW);
    
    //Wait conversion complete
    while (!(ADC_ChannelGetStatus(LPC_ADC0,ADC_CHANNEL_0,ADC_DATA_DONE)))
    {
        timeout--;
        if(timeout==0)
        {
            ADC_ChannelCmd(LPC_ADC0,ADC_CHANNEL_0,DISABLE);
            ADC_DeInit(LPC_ADC0);
            return 0;
        }
    }
    adc_value = ADC_ChannelGetData(LPC_ADC0,ADC_CHANNEL_0);
    
    ADC_ChannelCmd(LPC_ADC0,ADC_CHANNEL_0,DISABLE);
    
    ADC_DeInit(LPC_ADC0);

    adc_value*=5;
    
    return adc_value;
}

void action_shoot(uint8_t *cmd)
{
    active_weapons=read_type_arm(cmd+14);
    
    if(active_weapons==0)
    {
        cmd[0]=0x00;
        return;
    }
    
    switch(*cmd)
    {
    case 0x55:
        cmd[0]=action_shoot_forward();
        break;
    case 0xAA:
        cmd[0]=action_shoot_back();
        break;
    case 0x5A:
        cmd[0]=action_shoot_forward();
        if(cmd[0]==0xFF)
            cmd[0]=action_shoot_back();
        break;
    case 0xA5:
        cmd[0]=action_shoot_back();
        
        if(cmd[0]==0xFF)
            cmd[0]=action_shoot_forward();
        else
            if(cmd[0]==0xAA)
            {
                timer=1000;
                while(timer!=0);
                action_shoot_forward();
            }

        break;
    }
    
    CAN_Init( CAN_BITRATE1000K12MHZ, CLKDIV1 , TX_callback, RX_callback);// КОСТЫЛЬ. так как САN  и ADC0 висит на одной шине он дает збой
}

#define timeout_action 1500
#define kof_delta_u 10
#define count_elem_cur_null_test 20
#define max_searth_cur 1
#define min_searth_cur 0

uint16_t find_action_null_cur( uint8_t max_or_min)
{
	uint16_t cur_arr[count_elem_cur_null_test];
  uint16_t tmp;
	uint8_t i =0;
		
	i=count_elem_cur_null_test;
	while(i)
	{
    tmp=read_cur();
    if(tmp!=0)
		{
			i--;
			cur_arr[i] = tmp;
		}
	}
	
	tmp = cur_arr[0];	
	
	if(max_or_min == max_searth_cur)
	{
		for( i=1; i<count_elem_cur_null_test; i++)
			if(tmp < cur_arr[i])
				tmp = cur_arr[i];
	}
	else
	{
		for( i=1; i<count_elem_cur_null_test; i++)
			if(tmp > cur_arr[i])
				tmp = cur_arr[i];
	}
			
	return tmp;
}

uint8_t action_shoot_forward(void)
{
    uint16_t tmp;
    uint16_t cur_u=0;
    int32_t tmp_3=0;

	tmp = find_action_null_cur(min_searth_cur);
 
    GPIO_ClearValue(5, 1<<19);
    GPIO_ClearValue(5, 1<<17);
    
    GPIO_SetValue(5, 1<<19);
		
		timer = timeout_action;
		while(timer);
    
    timer=weapons_time[active_weapons-1]*1000-timeout_action;
    
    do
    {
S2:
        cur_u=read_cur();
        if(cur_u==0)
            goto S2;
        
        tmp_3=tmp-cur_u;
        if(tmp_3<0)
            tmp_3*=-1;
        
        if(tmp_3>weapons_max_cur[active_weapons-1])
        {
            GPIO_ClearValue(5, 1<<19);
            GPIO_ClearValue(5, 1<<17);
            return 0xAA;// Stop limit cur
        }
        
        if(timer==0)
        {
            GPIO_ClearValue(5, 1<<19);
            GPIO_ClearValue(5, 1<<17);
            return 0x55; // Stop limit time
        }
    }
    while(tmp>cur_u);
    
    GPIO_ClearValue(5, 1<<19);
    GPIO_ClearValue(5, 1<<17);
    
    return 0xFF;// Success
}

uint8_t action_shoot_back(void)
{
    uint16_t tmp;
    uint16_t cur_u=0;
    int32_t tmp_3=0;
	
	tmp = find_action_null_cur(max_searth_cur);

    GPIO_ClearValue(5, 1<<19);
    GPIO_ClearValue(5, 1<<17);
    
    GPIO_SetValue(5, 1<<17);
    
		timer = timeout_action;
		while(timer);
		
    timer=weapons_time[active_weapons-1]*1000 - timeout_action;
    
    do
    {
S2:
        cur_u=read_cur();
        if(cur_u==0)
            goto S2;
        
        tmp_3=tmp-cur_u;
        if(tmp_3<0)
            tmp_3*=-1;
        
        if(tmp_3>weapons_max_cur[active_weapons-1])
        {
            GPIO_ClearValue(5, 1<<19);
            GPIO_ClearValue(5, 1<<17);
            return 0xAA;// Stop limit cur
        }
        
        if(timer==0)
        {
            GPIO_ClearValue(5, 1<<19);
            GPIO_ClearValue(5, 1<<17);
            return 0x55;// Stop limit time
        }
    }
    while(tmp<cur_u);
    
    GPIO_ClearValue(5, 1<<19);
    GPIO_ClearValue(5, 1<<17);

    return 0xFF;
}

uint8_t read_type_arm(uint8_t *cmd)
{
    *cmd=0;
    
    if(!(GPIO_ReadValue(4) & (1<<9)))
        cmd[0]=1;
    
    if(!(GPIO_ReadValue(4) & (1<<8)))
        cmd[0]=2;
    
    if(!(GPIO_ReadValue(4) & (1<<8)) && !(GPIO_ReadValue(4) & (1<<9)))
        cmd[0]=3;
    
    return cmd[0];
}

void set_fier(uint8_t* com)
{
    if(com[0]==0x00)
    {
        GPIO_ClearValue(5, 1<<20);
        GPIO_ClearValue(5, 1<<21);
        com[0]=0xF0;
    }
    else
    {
        if(fire_on_flag==0)
        {
            com[0]=0x0F;
        }
        else
        {
            GPIO_SetValue(5, 1<<20);
            GPIO_SetValue(5, 1<<21);
            if(fire_time!=0)
            {
                fire_flag=1;
                timer=fire_time;
            }
            
            com[0]=0xFF;
        }
    }
}
