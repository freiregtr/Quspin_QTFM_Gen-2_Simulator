// quspin_gps_simulator.cpp
// Simulador monolítico para QuSpin v2 y GPS
// Compilar: g++ -std=c++11 -pthread quspin_gps_simulator.cpp -o quspin_gps_simulator -lutil
// Ejecutar: sudo ./quspin_gps_simulator
// Nota: Requiere permisos de root para crear dispositivos en /dev/

#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pty.h>
#include <cstring>
#include <atomic>
#include <random>
#include <mutex>
#include <signal.h>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdio>

// Variables globales para control
std::atomic<bool> running(true);
std::atomic<bool> identical_magnetometers(false);
std::atomic<bool> show_menu(true);
std::mutex print_mutex;

// Generadores de números aleatorios
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> noise_small(-0.1, 0.1);
std::uniform_real_distribution<> noise_medium(-1.0, 1.0);

// Estructura para datos del magnetómetro QuSpin
struct QuSpinData {
    // Datos escalares
    double scalar_field_nT;      // Campo magnético escalar |B| en nanoTesla
    char scalar_validation;      // '_' válido, '*' inválido

    // Datos vectoriales
    char vector_axis;            // 'X', 'Y', o 'Z'
    double vector_field_nT;      // Campo magnético vectorial en nanoTesla
    char vector_validation;      // '=' válido, '?' inválido

    // Metadatos
    uint16_t data_counter;       // 0-498 (incrementa de 2 en 2)
    uint32_t timestamp_ms;       // Timestamp en milisegundos (incrementa de 4 en 4)
    uint16_t scalar_sensitivity; // 0-999 (típicamente 50+)
    uint16_t vector_sensitivity; // 0-999 (típicamente 10+)
};

// Estructura para datos GPS
struct GPSData {
    double latitude;      // Grados decimales
    double longitude;     // Grados decimales
    double altitude;      // Metros
    double hdop;          // Dilución horizontal
    uint8_t satellites;   // Número de satélites
    uint8_t fix_quality;  // 0=sin fix, 1=GPS fix
    std::string utc_time; // HHMMSS.SS
};

// Valores base para simulación
struct SimulationValues {
    // Magnetómetro
    double base_scalar_field = 52930.0;  // nT típico para campo terrestre
    double base_vector_x = -785.0;
    double base_vector_y = 53000.0;
    double base_vector_z = 990.0;

    // GPS
    double base_latitude = 43.833357;   // 43°50.00141'N
    double base_longitude = -79.310330; // 079°18.61979'W
    double base_altitude = 208.7;
};

SimulationValues sim_values;

// Calcula checksum NMEA
std::string calculateNMEAChecksum(const std::string& sentence) {
    unsigned char checksum = 0;
    // El checksum se calcula entre $ y *
    for (size_t i = 1; i < sentence.length(); i++) {
        if (sentence[i] == '*') break;
        checksum ^= sentence[i];
    }

    std::stringstream ss;
    ss << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
       << static_cast<int>(checksum);
    return ss.str();
}

// Convierte decimal a grados y minutos decimales para NMEA
std::string decimalToNMEA(double decimal, bool isLatitude) {
    double absValue = std::abs(decimal);
    int degrees = static_cast<int>(absValue);
    double minutes = (absValue - degrees) * 60.0;

    std::stringstream ss;
    if (isLatitude) {
        ss << std::setfill('0') << std::setw(2) << degrees;
        ss << std::fixed << std::setprecision(5) << minutes;
    } else {
        ss << std::setfill('0') << std::setw(3) << degrees;
        ss << std::fixed << std::setprecision(5) << minutes;
    }

    return ss.str();
}

// Genera sentencia GNGGA
std::string generateGNGGA(const GPSData& data) {
    std::stringstream ss;
    ss << "$GNGGA," << data.utc_time << ",";
    ss << decimalToNMEA(data.latitude, true) << ",";
    ss << (data.latitude >= 0 ? "N" : "S") << ",";
    ss << decimalToNMEA(data.longitude, false) << ",";
    ss << (data.longitude >= 0 ? "E" : "W") << ",";
    ss << static_cast<int>(data.fix_quality) << ",";
    ss << std::setfill('0') << std::setw(2) << static_cast<int>(data.satellites) << ",";
    ss << std::fixed << std::setprecision(2) << data.hdop << ",";
    ss << std::fixed << std::setprecision(1) << data.altitude << ",M,";
    ss << "-36.0,M,,";

    std::string sentence = ss.str();
    sentence += "*" + calculateNMEAChecksum(sentence);

    return sentence;
}

// Genera sentencia GNZDA (aparece ocasionalmente)
std::string generateGNZDA(const std::string& utc_time) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm* tm_info = gmtime(&time_t);

    std::stringstream ss;
    ss << "$GNZDA," << utc_time << ",";
    ss << std::setfill('0') << std::setw(2) << tm_info->tm_mday << ",";
    ss << std::setfill('0') << std::setw(2) << (tm_info->tm_mon + 1) << ",";
    ss << (tm_info->tm_year + 1900) << ",00,00";

    std::string sentence = ss.str();
    sentence += "*" + calculateNMEAChecksum(sentence);

    return sentence;
}

// Genera línea de datos QuSpin
std::string generateQuSpinLine(const QuSpinData& data) {
    std::stringstream ss;

    // Campo escalar
    ss << "!" << std::fixed << std::setprecision(3) << data.scalar_field_nT
       << data.scalar_validation;

    // Campo vectorial
    ss << data.vector_axis;
    if (data.vector_field_nT >= 0) {
        ss << std::fixed << std::setprecision(3) << data.vector_field_nT;
    } else {
        // Para negativos, el signo ya está incluido
        ss << std::fixed << std::setprecision(3) << data.vector_field_nT;
    }
    ss << data.vector_validation;

    // Contador (3 dígitos con padding)
    ss << "@" << std::setfill('0') << std::setw(3) << data.data_counter;

    // Timestamp (sin padding específico)
    ss << ">" << data.timestamp_ms;

    // Sensibilidades (3 dígitos con padding)
    ss << "s" << std::setfill('0') << std::setw(3) << data.scalar_sensitivity;
    ss << "v" << std::setfill('0') << std::setw(3) << data.vector_sensitivity;

    return ss.str();
}

// Thread para emular GPS
void gpsEmulatorThread(int master_fd, const std::string& port_name) {
    GPSData gps_data;
    gps_data.latitude = sim_values.base_latitude;
    gps_data.longitude = sim_values.base_longitude;
    gps_data.altitude = sim_values.base_altitude;
    gps_data.hdop = 0.57;
    gps_data.satellites = 9;
    gps_data.fix_quality = 1;

    int time_counter = 0;
    int gnzda_counter = 0;

    // Tiempo inicial basado en ejemplo: 16:57:32.50
    int hours = 16;
    int minutes = 57;
    int seconds = 32;
    int centiseconds = 50;

    while (running) {
        // Actualizar tiempo UTC
        std::stringstream time_ss;
        time_ss << std::setfill('0') << std::setw(2) << hours
                << std::setfill('0') << std::setw(2) << minutes
                << std::setfill('0') << std::setw(2) << seconds
                << "." << std::setfill('0') << std::setw(2) << centiseconds;
        gps_data.utc_time = time_ss.str();

        // Pequeña variación en posición
        gps_data.latitude += noise_small(gen) * 0.000001;
        gps_data.longitude += noise_small(gen) * 0.000001;
        gps_data.altitude += noise_small(gen) * 0.1;

        // Generar sentencia GNGGA
        std::string nmea_sentence = generateGNGGA(gps_data) + "\r\n";

        // Escribir al puerto
        write(master_fd, nmea_sentence.c_str(), nmea_sentence.length());

        // Ocasionalmente enviar GNZDA (cada ~50 mensajes como en el ejemplo)
        gnzda_counter++;
        if (gnzda_counter >= 50) {
            std::string gnzda = generateGNZDA(gps_data.utc_time) + "\r\n";
            write(master_fd, gnzda.c_str(), gnzda.length());
            gnzda_counter = 0;
        }

        // Incrementar tiempo (0.1 segundos)
        centiseconds += 10;
        if (centiseconds >= 100) {
            centiseconds -= 100;
            seconds++;
            if (seconds >= 60) {
                seconds = 0;
                minutes++;
                if (minutes >= 60) {
                    minutes = 0;
                    hours++;
                    if (hours >= 24) {
                        hours = 0;
                    }
                }
            }
        }

        // GPS típicamente envía a 10Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Thread para emular magnetómetro QuSpin
void magnetometerEmulatorThread(int master_fd, const std::string& port_name, int mag_id) {
    static QuSpinData shared_data;  // Datos compartidos para modo idéntico
    static std::mutex shared_data_mutex;

    QuSpinData quspin_data;

    // Estado inicial
    uint16_t counter = 0;
    uint32_t timestamp = 86336800;  // Timestamp inicial del ejemplo
    char current_axis = 'X';

    // Valores base con pequeño offset entre magnetómetros si no son idénticos
    double offset = (mag_id == 1 && !identical_magnetometers) ? 10.0 : 0.0;

    while (running) {
        if (identical_magnetometers) {
            if (mag_id == 1) {
                // Magnetómetro 1 genera los datos
                quspin_data.scalar_field_nT = sim_values.base_scalar_field + noise_medium(gen);

                switch (current_axis) {
                    case 'X':
                        quspin_data.vector_field_nT = sim_values.base_vector_x + noise_medium(gen);
                        break;
                    case 'Y':
                        quspin_data.vector_field_nT = sim_values.base_vector_y + noise_medium(gen) * 10;
                        break;
                    case 'Z':
                        quspin_data.vector_field_nT = sim_values.base_vector_z + noise_medium(gen);
                        break;
                }

                quspin_data.scalar_validation = '_';
                quspin_data.vector_axis = current_axis;
                quspin_data.vector_validation = '=';
                quspin_data.data_counter = counter;
                quspin_data.timestamp_ms = timestamp;
                quspin_data.scalar_sensitivity = 135 + (rand() % 10);
                quspin_data.vector_sensitivity = 110 + (rand() % 10);

                // Guardar datos para mag2
                std::lock_guard<std::mutex> lock(shared_data_mutex);
                shared_data = quspin_data;
            } else {
                // Magnetómetro 2 usa los mismos datos
                std::this_thread::sleep_for(std::chrono::microseconds(100)); // Pequeño delay
                std::lock_guard<std::mutex> lock(shared_data_mutex);
                quspin_data = shared_data;
            }
        } else {
            // Modo independiente - cada magnetómetro genera sus propios datos
            quspin_data.scalar_field_nT = sim_values.base_scalar_field + offset + noise_medium(gen);

            switch (current_axis) {
                case 'X':
                    quspin_data.vector_field_nT = sim_values.base_vector_x + noise_medium(gen);
                    break;
                case 'Y':
                    quspin_data.vector_field_nT = sim_values.base_vector_y + noise_medium(gen) * 10;
                    break;
                case 'Z':
                    quspin_data.vector_field_nT = sim_values.base_vector_z + noise_medium(gen);
                    break;
            }

            quspin_data.scalar_validation = '_';
            quspin_data.vector_axis = current_axis;
            quspin_data.vector_validation = '=';
            quspin_data.data_counter = counter;
            quspin_data.timestamp_ms = timestamp;
            quspin_data.scalar_sensitivity = 135 + (rand() % 10);
            quspin_data.vector_sensitivity = 110 + (rand() % 10);
        }

        // Generar línea de datos
        std::string data_line = generateQuSpinLine(quspin_data) + "\n";

        // Escribir al puerto
        write(master_fd, data_line.c_str(), data_line.length());

        // Solo el mag1 actualiza contadores en modo idéntico
        if (!identical_magnetometers || mag_id == 1) {
            // Actualizar contadores
            counter += 2;
            if (counter > 498) {
                counter = 0;
            }

            timestamp += 4;

            // Rotar entre ejes X, Y, Z
            switch (current_axis) {
                case 'X': current_axis = 'Y'; break;
                case 'Y': current_axis = 'Z'; break;
                case 'Z': current_axis = 'X'; break;
            }
        }

        // QuSpin típicamente envía a ~250Hz (4ms entre muestras)
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
}

// Crear puerto serial virtual
int createVirtualPort(const std::string& symlink_path) {
    int master_fd, slave_fd;
    char slave_name[256];

    // Verificar si el archivo existe y hacer backup si es necesario
    struct stat st;
    if (stat(symlink_path.c_str(), &st) == 0) {
        std::cout << "ADVERTENCIA: " << symlink_path << " ya existe." << std::endl;

        // Si es un dispositivo real (character device), hacer backup
        if (S_ISCHR(st.st_mode)) {
            std::string backup_path = symlink_path + ".backup";
            std::cout << "Es un dispositivo real. Renombrando a " << backup_path << std::endl;
            rename(symlink_path.c_str(), backup_path.c_str());
        } else {
            // Si es un symlink o archivo, simplemente eliminarlo
            unlink(symlink_path.c_str());
        }
    }

    // Abrir pseudo-terminal
    if (openpty(&master_fd, &slave_fd, slave_name, NULL, NULL) == -1) {
        std::cerr << "Error al crear pty: " << strerror(errno) << std::endl;
        return -1;
    }

    // Configurar el puerto como non-blocking
    int flags = fcntl(master_fd, F_GETFL, 0);
    fcntl(master_fd, F_SETFL, flags | O_NONBLOCK);

    // Crear symlink
    if (symlink(slave_name, symlink_path.c_str()) == -1) {
        std::cerr << "Error al crear symlink " << symlink_path << ": "
                  << strerror(errno) << std::endl;
        std::cerr << "¿Estás ejecutando con sudo?" << std::endl;
        close(master_fd);
        close(slave_fd);
        return -1;
    }

    // Configurar permisos
    chmod(symlink_path.c_str(), 0666);

    // Si es GPS, configurar baudrate a 9600
    if (symlink_path.find("AMA0") != std::string::npos) {
        struct termios tty;
        tcgetattr(slave_fd, &tty);
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tcsetattr(slave_fd, TCSANOW, &tty);
    }

    std::cout << "Puerto virtual creado: " << symlink_path
              << " -> " << slave_name << std::endl;

    close(slave_fd);  // No necesitamos el slave
    return master_fd;
}

// Manejador de señal para salida limpia
void signalHandler(int signum) {
    std::cout << "\nRecibida señal " << signum << ". Terminando..." << std::endl;
    running = false;
}

// Mostrar menú de control
void showControlMenu() {
    std::cout << "\n=== SIMULADOR QUSPIN v2 Y GPS ===" << std::endl;
    std::cout << "Puertos virtuales activos:" << std::endl;
    std::cout << "  - GPS:          /dev/ttyAMA0" << std::endl;
    std::cout << "  - Magnetómetro 1: /dev/ttyAMA2" << std::endl;
    std::cout << "  - Magnetómetro 2: /dev/ttyAMA4" << std::endl;
    std::cout << "\nComandos:" << std::endl;
    std::cout << "  i - Toggle magnetómetros idénticos/Y-splitter (actual: "
              << (identical_magnetometers ? "SÍ - IDÉNTICOS" : "NO - INDEPENDIENTES") << ")" << std::endl;
    std::cout << "  m - Mostrar este menú" << std::endl;
    std::cout << "  q - Salir" << std::endl;
    std::cout << "\nConfiguración actual:" << std::endl;
    std::cout << "  - GPS: 9600 baud, 8N1" << std::endl;
    std::cout << "  - Magnetómetros: 115200 baud, 8N1" << std::endl;
    std::cout << "  - Datacount: 0-498 (incrementa de 2 en 2)" << std::endl;
    std::cout << "  - Timestamp: incrementa de 4 en 4 ms" << std::endl;
    std::cout << "\nPara probar en otra terminal:" << std::endl;
    std::cout << "  GPS:  screen /dev/ttyAMA0 9600" << std::endl;
    std::cout << "  MAG1: screen /dev/ttyAMA2 115200" << std::endl;
    std::cout << "  MAG2: screen /dev/ttyAMA4 115200" << std::endl;
    std::cout << "================================\n" << std::endl;
}

// Thread para manejar entrada del usuario
void userInputThread() {
    std::string input;
    while (running) {
        if (show_menu) {
            showControlMenu();
            show_menu = false;
        }

        std::getline(std::cin, input);

        if (input == "q") {
            running = false;
        } else if (input == "i") {
            identical_magnetometers = !identical_magnetometers;
            std::cout << "\n*** Magnetómetros configurados como: "
                      << (identical_magnetometers ? "IDÉNTICOS (Y-splitter)" : "INDEPENDIENTES")
                      << " ***" << std::endl;
            if (identical_magnetometers) {
                std::cout << "Ambos magnetómetros ahora emiten exactamente los mismos datos." << std::endl;
            } else {
                std::cout << "Cada magnetómetro genera datos independientes con ruido propio." << std::endl;
            }
            std::cout << std::endl;
        } else if (input == "m") {
            show_menu = true;
        }
    }
}

int main() {
    // Verificar si se ejecuta como root
    if (geteuid() != 0) {
        std::cerr << "Este programa necesita permisos de root para crear dispositivos en /dev/" << std::endl;
        std::cerr << "Por favor ejecuta con: sudo ./quspin_gps_simulator" << std::endl;
        return 1;
    }

    // Instalar manejador de señal
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== INICIANDO SIMULADOR EN RASPBERRY PI 5 ===" << std::endl;
    std::cout << "NOTA: Este simulador creará puertos virtuales en:" << std::endl;
    std::cout << "  /dev/ttyAMA0 (GPS)" << std::endl;
    std::cout << "  /dev/ttyAMA2 (Magnetómetro 1)" << std::endl;
    std::cout << "  /dev/ttyAMA4 (Magnetómetro 2)" << std::endl;
    std::cout << "\nSi tienes hardware real conectado, este será temporalmente deshabilitado." << std::endl;
    std::cout << "Los dispositivos originales serán restaurados al salir del simulador.\n" << std::endl;
    std::cout << "Presiona ENTER para continuar o Ctrl+C para cancelar..." << std::endl;
    std::cin.get();
    std::cout << "Creando puertos virtuales..." << std::endl;

    // Crear puertos virtuales
    int gps_fd = createVirtualPort("/dev/ttyAMA0");
    int mag1_fd = createVirtualPort("/dev/ttyAMA2");
    int mag2_fd = createVirtualPort("/dev/ttyAMA4");

    if (gps_fd == -1 || mag1_fd == -1 || mag2_fd == -1) {
        std::cerr << "Error al crear puertos virtuales" << std::endl;
        return 1;
    }

    // Mostrar menú inicial
    show_menu = true;

    // Crear threads
    std::thread gps_thread(gpsEmulatorThread, gps_fd, "/dev/ttyAMA0");
    std::thread mag1_thread(magnetometerEmulatorThread, mag1_fd, "/dev/ttyAMA2", 1);
    std::thread mag2_thread(magnetometerEmulatorThread, mag2_fd, "/dev/ttyAMA4", 2);
    std::thread input_thread(userInputThread);

    // Esperar a que terminen los threads
    gps_thread.join();
    mag1_thread.join();
    mag2_thread.join();
    input_thread.join();

    // Limpiar
    close(gps_fd);
    close(mag1_fd);
    close(mag2_fd);

    std::cout << "\nLimpiando puertos virtuales..." << std::endl;

    // Eliminar symlinks
    unlink("/dev/ttyAMA0");
    unlink("/dev/ttyAMA2");
    unlink("/dev/ttyAMA4");

    // Restaurar backups si existen
    struct stat st;
    if (stat("/dev/ttyAMA0.backup", &st) == 0) {
        rename("/dev/ttyAMA0.backup", "/dev/ttyAMA0");
        std::cout << "Restaurado /dev/ttyAMA0 original" << std::endl;
    }
    if (stat("/dev/ttyAMA2.backup", &st) == 0) {
        rename("/dev/ttyAMA2.backup", "/dev/ttyAMA2");
        std::cout << "Restaurado /dev/ttyAMA2 original" << std::endl;
    }
    if (stat("/dev/ttyAMA4.backup", &st) == 0) {
        rename("/dev/ttyAMA4.backup", "/dev/ttyAMA4");
        std::cout << "Restaurado /dev/ttyAMA4 original" << std::endl;
    }

    std::cout << "Simulador terminado." << std::endl;

    return 0;
}