HardwareSerial GpsSerial(2);

#define GPS_RX 16
#define GPS_TX 17

#define EARTH_RADIUS 6371000.0

// ============================================================
// DADOS DO GPS
// ============================================================

float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;

int satellites = 0;

float velocity_kmh = 0.0;
float velocity_ms = 0.0;

float acceleration = 0.0;

double distance = 0.0;


// ============================================================
// DADOS ANTERIORES
// ============================================================

float previousLatitude = 0.0;
float previousLongitude = 0.0;

float previousVelocity = 0.0;

unsigned long previousTime = 0;

bool firstPosition = true;
bool firstVelocity = true;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    GpsSerial.begin(
        9600,
        SERIAL_8N1,
        GPS_RX,
        GPS_TX
    );

    Serial.println("AT6558 iniciado!");
    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    static String sentence = "";

    while (GpsSerial.available() > 0)
    {
        char c = GpsSerial.read();

        // Começo de uma sentença NMEA
        if (c == '$')
        {
            sentence = "$";
        }

        else if (sentence.length() > 0)
        {
            sentence += c;
        }

        // Final da sentença
        if (c == '\n')
        {
            processNMEA(sentence);

            sentence = "";
        }
    }
}


// ============================================================
// PROCESSAMENTO NMEA
// ============================================================

void processNMEA(String sentence)
{
    sentence.trim();

    // ========================================================
    // GNGGA
    // Latitude
    // Longitude
    // Altitude
    // Satélites
    // ========================================================

    if (sentence.startsWith("$GNGGA"))
    {
        processGGA(sentence);
    }


    // ========================================================
    // GNVTG
    // Velocidade
    // ========================================================

    else if (sentence.startsWith("$GNVTG"))
    {
        processVTG(sentence);
    }
}


// ============================================================
// PROCESSA GGA
// ============================================================

void processGGA(String sentence)
{
    String fields[15];

    int field = 0;
    int start = 0;

    for (int i = 0; i < sentence.length(); i++)
    {
        if (sentence[i] == ',')
        {
            if (field < 15)
            {
                fields[field] =
                    sentence.substring(start, i);

                field++;
            }

            start = i + 1;
        }
    }

    if (field < 15)
    {
        fields[field] =
            sentence.substring(start);
    }


    // --------------------------------------------------------
    // Verifica FIX
    // --------------------------------------------------------

    int fixQuality =
        fields[6].toInt();

    if (fixQuality == 0)
    {   
        Serial.println("Sem sinal de SAT.");
        return;
    }


    // --------------------------------------------------------
    // Satélites
    // --------------------------------------------------------

    satellites =
        fields[7].toInt();


    // --------------------------------------------------------
    // Latitude
    // --------------------------------------------------------

    float rawLatitude =
        fields[2].toFloat();

    latitude =
        convertCoordinate(
            rawLatitude,
            fields[3][0]
        );


    // --------------------------------------------------------
    // Longitude
    // --------------------------------------------------------

    float rawLongitude =
        fields[4].toFloat();

    longitude =
        convertCoordinate(
            rawLongitude,
            fields[5][0]
        );


    // --------------------------------------------------------
    // Altitude
    // --------------------------------------------------------

    altitude =
        fields[9].toFloat();


    // ========================================================
    // DISTÂNCIA
    // ========================================================

    if (!firstPosition)
    {
        double deltaDistance =
            calculateDistance(
                previousLatitude,
                previousLongitude,
                latitude,
                longitude
            );


        /*
         * Ignora saltos absurdos do GPS.
         *
         * Se o GPS perder o sinal momentaneamente,
         * pode aparecer uma posição muito distante.
         */

        if (deltaDistance < 100.0)
        {
            distance += deltaDistance;
        }
    }


    previousLatitude =
        latitude;

    previousLongitude =
        longitude;

    firstPosition = false;
}


// ============================================================
// PROCESSA VTG
// ============================================================

void processVTG(String sentence)
{
    String fields[12];

    int field = 0;
    int start = 0;

    for (int i = 0; i < sentence.length(); i++)
    {
        if (sentence[i] == ',')
        {
            if (field < 12)
            {
                fields[field] =
                    sentence.substring(start, i);

                field++;
            }

            start = i + 1;
        }
    }

    if (field < 12)
    {
        fields[field] =
            sentence.substring(start);
    }


    /*
     * Estrutura:

       $GNVTG,
       1 curso verdadeiro
       2 T
       3 curso magnético
       4 M
       5 velocidade em nós
       6 N
       7 velocidade km/h
       8 K
       ...
    */


    // --------------------------------------------------------
    // Velocidade em km/h
    // --------------------------------------------------------

    if (fields[7].length() == 0)
    {
        return;
    }


    float newVelocityKmh =
        fields[7].toFloat();


    float newVelocityMs =
        newVelocityKmh / 3.6;


    // ========================================================
    // ACELERAÇÃO
    // ========================================================

    unsigned long currentTime =
        millis();


    if (!firstVelocity)
    {
        float dt =
            (currentTime - previousTime)
            / 1000.0;


        if (dt > 0.0)
        {
            float rawAcceleration =
                (newVelocityMs - previousVelocity)
                / dt;


            /*
             * Filtro passa-baixas simples.
             *
             * Reduz a variação causada pelo
             * ruído do GNSS.
             */

            const float alpha = 0.2;

            acceleration =
                alpha * rawAcceleration
                +
                (1.0 - alpha) * acceleration;
        }
    }


    previousVelocity =
        newVelocityMs;

    previousTime =
        currentTime;

    firstVelocity = false;


    velocity_kmh =
        newVelocityKmh;

    velocity_ms =
        newVelocityMs;


    // ========================================================
    // MOSTRA TUDO
    // ========================================================

    printData();
}


// ============================================================
// MOSTRA DADOS
// ============================================================

void printData()
{
    Serial.println();
    Serial.println("--------------------------------");

    Serial.print("Latitude: ");
    Serial.print(latitude, 6);
    Serial.println(" deg");

    Serial.print("Longitude: ");
    Serial.print(longitude, 6);
    Serial.println(" deg");

    Serial.print("Altitude: ");
    Serial.print(altitude, 2);
    Serial.println(" m");

    Serial.print("Satelites: ");
    Serial.println(satellites);

    Serial.print("Velocidade: ");
    Serial.print(velocity_kmh, 2);
    Serial.println(" km/h");

    Serial.print("Velocidade: ");
    Serial.print(velocity_ms, 2);
    Serial.println(" m/s");

    Serial.print("Distancia: ");
    Serial.print(distance, 2);
    Serial.println(" m");

    Serial.print("Aceleracao: ");
    Serial.print(acceleration, 3);
    Serial.println(" m/s^2");

    Serial.println("--------------------------------");
}


// ============================================================
// CONVERSÃO NMEA -> GRAUS DECIMAIS
// ============================================================

float convertCoordinate(
    float coordinate,
    char direction
)
{
    int degrees =
        (int)(coordinate / 100.0);

    float minutes =
        coordinate -
        degrees * 100.0;

    float decimal =
        degrees +
        minutes / 60.0;


    if (
        direction == 'S' ||
        direction == 'W'
    )
    {
        decimal *= -1;
    }

    return decimal;
}


// ============================================================
// DISTÂNCIA HAVERSINE
// ============================================================

double calculateDistance(
    float lat1,
    float lon1,
    float lat2,
    float lon2
)
{
    double lat1Rad =
        lat1 * PI / 180.0;

    double lat2Rad =
        lat2 * PI / 180.0;

    double deltaLat =
        (lat2 - lat1) * PI / 180.0;

    double deltaLon =
        (lon2 - lon1) * PI / 180.0;


    double a =
        sin(deltaLat / 2.0) *
        sin(deltaLat / 2.0)
        +
        cos(lat1Rad) *
        cos(lat2Rad) *
        sin(deltaLon / 2.0) *
        sin(deltaLon / 2.0);


    double c =
        2.0 *
        atan2(
            sqrt(a),
            sqrt(1.0 - a)
        );


    return EARTH_RADIUS * c;
}