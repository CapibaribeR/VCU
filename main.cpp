#include "mbed.h"
#include "MPU9250.h"
#include "time.h"

//Define as portas de Entrada
AnalogIn volante (PF_10);
AnalogIn sinal_in(PF_5);    //BSE
AnalogIn apps1(PA_0);
AnalogIn apps2(PF_4);
AnalogOut saida(PA_5);     //APPS (via CAN)
InterruptIn sensor1(PA_4, PullNone);     //Velocidade 1
//InterruptIn sensor2(PA_5, PullNone);   //Velocidade 2

//Declaração de Variáveis   
//BSE
int valor_entrada = 0; //irá receber a entrada do sensor
int valor_max = 45500; //valor máximo atingido pelo sensor durante os testes 
int valor_min = 6129; //valor mínimo atingido pelo sensor durante os testes
int angulo; //saída formatada em ângulo(0° - 120°)
unsigned long int prevTime = 0; //váriaveis para contagem de tempo
unsigned long int prevTime1 = 0; 
double erro_min = 4500; //limite inferior do valor_entrada para que não seja erro
double erro_max = 48000; //limite superior do valor_entrada para que não seja erro
bool erro = false; //varável que guarda a informação caso ocorra algum erro
//APPS
int cont = 0;                      // Variável para denotar o início ou fim da contagem de tempo
unsigned long jikan = 0.0;         // Variável para guardar o número de milissegundos 
float aux = 0.0;                   // Variável para guardar o torque
int erro_apps = 0;                 // Variável para denotar a existência de erro
//Velocidade 1
//Timer t;  (já é usado no MPU)
uint8_t contagem_pulso1 = 0;
uint64_t periodo_atual1 = 0, ultimo_t1 = 0;
float f1 = 0;
/*Velocidade 2
Timer t;  (já é usado no MPU)
uint8_t contagem_pulso2 = 0;
uint64_t periodo_atual2 = 0, ultimo_t2 = 0;
float f2 = 0;*/

MPU9250 mpu9250;

Timer t;

//Funções
//BSE
//Mesma função map do arduino
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//Função millis() do arduino para contagem de tempo ()ver se não vai dar problema
double millis(){
    using namespace std::chrono;
    auto now_ms = time_point_cast<microseconds>(Kernel::Clock::now());
    long micros = now_ms.time_since_epoch().count();
    return micros / 1000.0;
}
//Verificar se houve um curto ou se foi aberto o circuito do sensor
void Verifica_Erro(int entrada){
  if(entrada < erro_min or entrada > erro_max){
    if(millis() - prevTime >= 100){
       prevTime =  millis();
       erro = true;
    } 
  }
  else{
    prevTime = millis();
  }
}

//APPS
// Função para verificar se ocorreu erros nos torques (Se os torques 
// se tiverem uma diferença de 10% durando 100 milissegundos, será erro)
int VerificaErroTorque(float t1, float t2) {
  if (abs(t1 - t2) > (0.1 * max(t1, t2))){
    cont += 1;
    if(cont == 1) {
      jikan = millis(); 
    }
    if(millis() - jikan > 100) {
      return 0;
    }
    return 1;
  } else{
      cont = 0;
      return 1;
  }
}

//Velocidade 1
void contadorFrequenciaISR1(){
    contagem_pulso1++;                                     // Quantidade de pulsos que aconteceram
    periodo_atual1 += t.elapsed_time().count() - ultimo_t1; // Tamanho do pulso em s
    ultimo_t1 = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}

/*Velocidade 2
void contadorFrequenciaISR2(){
    contagem_pulso2++;                                     // Quantidade de pulsos que aconteceram
    periodo_atual2 += t.elapsed_time().count() - ultimo_t2; // Tamanho do pulso em s
    ultimo_t2 = t.elapsed_time().count();                  // Renicializa a contagem do tempo     
}*/

int main() {
    i2c.frequency(400000);  // Inicializa um I2C rápido (400Hz)  
  
    t.start();

    mpu9250.resetMPU9250(); // Reinicia os registradores para o default para preparar para a calibração do dispositivo
    mpu9250.MPU9250SelfTest(SelfTest); // Começa realizando um teste inicial e reporta os valores
    mpu9250.initMPU9250(); // Inicializa o MPU

    mpu9250.getAres(); // Sensitividade do acelerômetro
    mpu9250.getGres(); // Sensitividade do giroscópio

    while(true) {
        // Se o pino for para HIGh, todo os registradores recebem novos dados
        if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // Na interrupção, checa se a data está prota para interromper
            
            mpu9250.readAccelData(accelCount);  // Lê os valores x/y/z do adc   
            // Calcula a aceleração em g's
            ax = (float)accelCount[0] * aRes - accelBias[0];
            ay = (float)accelCount[1] * aRes - accelBias[1];   
            az = (float)accelCount[2] * aRes - accelBias[2];  
    
            mpu9250.readGyroData(gyroCount);  // Lê os valores x/y/z do adc  
            // Calcula a velocidade angular em graus por segundo
            gx = (float)gyroCount[0] * gRes - gyroBias[0];
            gy = (float)gyroCount[1] * gRes - gyroBias[1];  
            gz = (float)gyroCount[2] * gRes - gyroBias[2];   
        }

        // Normalização e conversão dos valores obtidos
        mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, 0, 0, 0);

        // Printa os dados de aceleração convertendo para m/s² e fazendo a conversão
        printf("ax = %f", ax * 9.81 - 0.15); 
        printf(" ay = %f", ay * 9.81 - 0.1); 
        printf(" az = %f  m/s²\n\r", az * 9.81 + 0.12);

        // Printa os dados de velocidade angular
        printf("gx = %f", gx); 
        printf(" gy = %f", gy); 
        printf(" gz = %f  rad/s\n\r", gz); 

        // Lê o valores adc de temperatura em Kelvin, converte para graus Celsius e printa
        tempCount = mpu9250.readTempData();
        temperature = ((float) tempCount) / 333.87f + 21.0f;
        printf(" temperature = %f  C\n\r", temperature);

        //VOLANTE
        //Lê o valor do sensor (entre 0 65535) e converte para valores entre 0 e 3.3
        float valor = volante.read_u16() * (3.3 / 65535.0);
        //Centraliza os valores
        float centro = (valor - 1.6);
        //Printa valor
        //printf("%.2f\n", valor);
        //Printa centro
        printf("%.2f\n", centro);
        //Se centro for maior que 0
        if(centro > 0){
            printf("Direita\n");
        }
        //Se centro for menor que 0
        if(centro < 0){
            printf("Esquerda\n");
        }

        //BSE
        valor_entrada = sinal_in.read_u16();

        Verifica_Erro(valor_entrada);

        angulo = map(valor_entrada,valor_min, valor_max, 0, 120);

        if(erro){
            printf("ERRO\n");
        }
        else{
            if(millis() - prevTime1 >= 200){
                printf("Valor entrada: ");
                printf("%d\n", valor_entrada);
                printf("Max: ");
                printf("%d\n", valor_max);
                printf("Min: ");
                printf("%d\n", valor_min);
                printf("Angulo: ");
                printf("%d\n", angulo);
                prevTime1 = millis();
            }
        }
    
        //APPS
        // Determina a tensão que será enviada para o inversor (read_u16 devolve valor entre 0 e 65535)
        int torque1 = apps1.read_u16();
        int torque2 = apps2.read_u16();
        int maxi = max(torque1, torque2);

        printf("%d\n", torque1);    //teste serial
        printf("%d\n", torque2);    //teste serial
        printf("%d\n", maxi);   //teste serial

        // Verifica Erros e Envia o Torque, se acontecerem os erros, deve mandar torque 0 para o inversor
        if (VerificaErroTorque(torque1, torque2) == 0){
            // Aconteceu erro (torques com mais de 10% de diferença por 100 mseg ou mais)
            saida.write_u16(0);
            aux = 0;
        } else{
            // Como o erro não aconteceu, o maior torque entre os sensores deve ser enviado para o inversor,
            // porém, como o inversor não funciona se o torque inicial não começar no mínimo aceitável por
            // ele, é preciso que o torque aumente ou diminua gradativamente até o torque desejado
            if (aux < maxi) {
                // aux deve ir aumentado gradativamente até o máximo, mas se der erro enquanto isso, deverá parar
                while ((aux < maxi) && (erro_apps == 0)){
                    if (VerificaErroTorque(apps1.read_u16(), apps2.read_u16()) == 0){
                    // Aconteceu erro durante
                    saida.write_u16(0);
                    aux = 0;
                    erro_apps = 1;
                    } else{
                        // Manda o torque para o inversor, indo até o maior dos dois torques
                        saida.write_u16(aux);
                        aux = aux + 0.5;
                    }
                }
                aux = max(apps1.read_u16(), apps2.read_u16());
                saida.write_u16(aux);
            } 
            if (aux > maxi) {
                // aux deve ir diminuindo gradativamente até o máximo, mas se der erro enquanto isso, deverá parar
                while ((aux > maxi) && (erro_apps == 0)){
                    if (VerificaErroTorque(apps1.read_u16(), apps2.read_u16()) == 0){
                        // Aconteceu erro durante
                        saida.write_u16(0);
                        aux = 0;
                        erro_apps = 1;
                    } else{
                    // Manda o torque para o inversor, indo até o maior dos dois torques
                    saida.write_u16(aux);
                    aux = aux - 0.5;
                    }
                }
            }
            erro_apps = 0;
            float vari = saida;     //teste serial
            printf("%.2f\n", vari);     //teste serial
            printf("Recomeço\n");
        }

        //Velocidade 1
        sensor1.fall(NULL); // Desativa o interrupt
        if (periodo_atual1 != 0){
            f1 = 1000000 * ((float)(contagem_pulso1)/periodo_atual1); // Calcula a frequência em Hz
        }
        else{
            f1 = 0;
        }

        // Reinicialização dos parâmetro do pulso
        contagem_pulso1 = 0;                          
        periodo_atual1 = 0;                  
        ultimo_t1 = t.elapsed_time().count();        
        sensor1.fall(&contadorFrequenciaISR1); // Ativa o interrupt
        
        printf("Velocidade 1\n");
        printf("%d\n", int(f1 * 30)); // Converte a velocidade para RPM
        
        /*Velocidade 2
        sensor2.fall(NULL); // Desativa o interrupt
        if (periodo_atual2 != 0){
            f2 = 1000000 * ((float)(contagem_pulso2)/periodo_atual2); // Calcula a frequência em Hz
        }
        else{
            f2 = 0;
        }

        // Reinicialização dos parâmetro do pulso
        contagem_pulso2 = 0;                          
        periodo_atual2 = 0;                  
        ultimo_t2 = t.elapsed_time().count();        
        sensor2.fall(&contadorFrequenciaISR2); // Ativa o interrupt
        
        printf("Velocidade 2\n");
        printf("%d\n", int(f2 * 30)); // Converte a velocidade para RPM*/

        wait_us(500000);
    }
}
