HardwareSerial GpsSerial(2);

#define GPS_RX 16
#define GPS_TX 17

// Variáveis do GPS
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;

int satellites = 0;

void setup()
{
    Serial.begin(115200);

    // UART2
    // RX = GPIO16
    // TX = GPIO17
    GpsSerial.begin(
        9600,
        SERIAL_8N1,
        GPS_RX,
        GPS_TX
    );

    Serial.println("AT6558 iniciado!");
    Serial.println();
}

void loop()
{
    static String sentence = "";

    while (GpsSerial.available() > 0)
    {
        char c = GpsSerial.read();

        // Começo de uma mensagem NMEA
        if (c == '$')
        {
            sentence = "$";
        }

        // Continua armazenando a mensagem
        else if (sentence.length() > 0)
        {
            sentence += c;
        }

        // Final da mensagem
        if (c == '\n')
        {
            processNMEA(sentence);

            sentence = "";
        }
    }
}


// ============================================================
// PROCESSA MENSAGEM NMEA
// ============================================================

void processNMEA(String sentence)
{
    // Estamos interessados no GNGGA
    if (!sentence.startsWith("$GNGGA"))
    {
        return;
    }

    // Remove \r e \n
    sentence.trim();

    // Divide a mensagem em campos
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

    // Último campo
    if (field < 15)
    {
        fields[field] =
            sentence.substring(start);
    }


    /*
       Estrutura:

       $GNGGA,
       1 UTC
       2 Latitude
       3 N/S
       4 Longitude
       5 E/W
       6 Fix quality
       7 Número de satélites
       8 HDOP
       9 Altitude
       10 M
       ...
    */


    // --------------------------------------------------------
    // Verifica se existe FIX
    // --------------------------------------------------------

    int fixQuality =
        fields[6].toInt();

    if (fixQuality == 0)
    {
        Serial.println("Sem FIX GPS");
        return;
    }


    // --------------------------------------------------------
    // Número de satélites
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


    // --------------------------------------------------------
    // MOSTRA OS RESULTADOS
    // --------------------------------------------------------

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

    Serial.print("Satélites: ");
    Serial.println(satellites);

    Serial.println("--------------------------------");
}


// ============================================================
// CONVERTE COORDENADA NMEA
// ============================================================

float convertCoordinate(
    float coordinate,
    char direction
)
{
    /*
       NMEA utiliza:

       Latitude:
       DDMM.MMMM

       Longitude:
       DDDMM.MMMM

       Exemplo:

       2312.3456

       = 23 graus
       + 12.3456 minutos
    */

    int degrees =
        (int)(coordinate / 100.0);

    float minutes =
        coordinate -
        (degrees * 100.0);

    float decimal =
        degrees +
        minutes / 60.0;


    // Sul e Oeste são negativos
    if (
        direction == 'S' ||
        direction == 'W'
    )
    {
        decimal *= -1;
    }

    return decimal;
}