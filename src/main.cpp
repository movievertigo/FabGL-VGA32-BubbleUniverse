#include "fabgl.h"
#include <esp_task_wdt.h>

#define DS_320x240_60Hz   "\"320x240@60d\" 12.14 320 352 368 400 240 245 248 253 -HSync -VSync DoubleScan"
#define DS_320x240_75Hz   "\"320x240@75d\" 15.48 320 352 376 408 240 245 248 253 -HSync -VSync DoubleScan"
#define DS_320x240_85Hz   "\"320x240@85d\" 17.89 320 352 384 416 240 244 248 253 -HSync -VSync DoubleScan"
#define DS_320x240_100Hz "\"320x240@100d\" 21.46 320 352 392 424 240 244 248 253 -HSync -VSync DoubleScan"
#define DS_320x240_120Hz "\"320x240@120d\" 26.23 320 352 400 432 240 244 249 253 -HSync -VSync DoubleScan"
#define DS_320x240_150Hz "\"320x240@150d\" 34.00 320 352 416 448 240 243 249 253 -HSync -VSync DoubleScan"
#define SS_320x240_60Hz    "\"320x240@60\" 06.07 320 352 368 400 240 245 248 253 -HSync -VSync"
#define SS_320x240_75Hz    "\"320x240@75\" 07.74 320 352 376 408 240 245 248 253 -HSync -VSync"
#define SS_320x240_85Hz    "\"320x240@85\" 08.95 320 352 384 416 240 244 248 253 -HSync -VSync"
#define SS_320x240_100Hz  "\"320x240@100\" 10.73 320 352 392 424 240 244 248 253 -HSync -VSync"
#define SS_320x240_120Hz  "\"320x240@120\" 13.12 320 352 400 432 240 244 249 253 -HSync -VSync"
#define SS_320x240_150Hz  "\"320x240@150\" 17.00 320 352 416 448 240 243 249 253 -HSync -VSync"
#define SS_320x240_160Hz  "\"320x240@160\" 18.14 320 352 416 448 240 243 250 253 -HSync -VSync"


const char* screenMode = SS_320x240_160Hz;
const int pixelSize = 240;
const float pSize = 0.25f * pixelSize;
const float pOffset = 0.5f * pixelSize;
const int sinTableSize = 1<<13;
const float sinTableScale = sinTableSize / (2*PI) / pSize;

float* sinTable;
uint8_t* offscreen;

fabgl::VGAController displayController;
fabgl::Canvas canvas(&displayController);
fabgl::Bitmap bitmap;
SemaphoreHandle_t semaphore;

const int curveShift = 8;
const int curveCount = 1<<curveShift;
const int curveStep = 4;
const int iterationShift = 9;
const int iterationCount = 1<<iterationShift;
float speed = 0.00015f;

const float ang1inc = pSize * curveStep * 2 * PI / 235;
const float ang2inc = pSize * curveStep;
float animTime;

int frame = 0;
int totalTime = 0;

void createSinTable()
{
    sinTable = (float*)malloc(sinTableSize*1.25 * sizeof(float));

    if (sinTable)
    {
        for (int i = 0; i < sinTableSize/4; ++i)
        {
            float value = sin(i * 2 * PI / sinTableSize) * pSize; // pre-multiplied by pSize (and offset below) so we don't need to do it later
            sinTable[i] = pOffset * 0.5f + value;
            sinTable[sinTableSize/2-i-1] = pOffset * 0.5f + value;
            sinTable[sinTableSize/2+i] = pOffset * 0.5f - value;
            sinTable[sinTableSize-i-1] = pOffset * 0.5f - value;
            sinTable[sinTableSize+i] = pOffset * 0.5f + value;
        }
    }
    else
    {
        Serial.println("Failed to allocate memory for sin table");
    }
}

void setup(void)
{
    Serial.begin(115200);

    displayController.begin();
    displayController.setResolution(screenMode, pixelSize, pixelSize, false);

    canvas.selectFont(&fabgl::FONT_8x8);
    canvas.setBrushColor(Color::Black);
    canvas.setPenColor(Color::White);

    offscreen = (uint8_t*)malloc(pixelSize * pixelSize * sizeof(uint8_t));
    createSinTable();

    bitmap.width = pixelSize;
    bitmap.height = pixelSize;
    bitmap.data = offscreen;
    bitmap.format = PixelFormat::RGBA2222;

    semaphore = xSemaphoreCreateBinary();
}

void bubbleUniverseHalf(int section)
{
    const int iStart = section * curveCount / 2;
    const int iEnd = (1 + section) * curveCount / 2;
    float ang1Start = animTime * pSize + section * ang1inc * curveCount / curveStep / 2 - pOffset;
    float ang2Start = animTime * pSize + section * ang2inc * curveCount / curveStep / 2 - pOffset;

    for (int i = iStart; i < iEnd; i += curveStep)
    {
        const uint8_t red = ((i+(0xC0<<(curveShift-2)))>>(curveShift-2));

        float x = pOffset, y = pOffset;
        for (int j = 0; j < iterationCount; ++j)
        {
            const int sinoffset1 = (int)((ang1Start + x) * sinTableScale) & (sinTableSize-1);
            const int sinoffset2 = (int)((ang2Start + y) * sinTableScale) & (sinTableSize-1);

            x = sinTable[sinoffset1] + sinTable[sinoffset2];
            y = sinTable[sinoffset1+sinTableSize/4] + sinTable[sinoffset2+sinTableSize/4];
            const uint8_t green = j>>(iterationShift-2);
            const uint8_t blue = (7-(red+green))>>1;
            offscreen[(int)y*pixelSize + (int)x] = red + (green << 2) + (blue << 4);
        }

        ang1Start += ang1inc;
        ang2Start += ang2inc;
    }
}

void halfTask(void* params)
{
  esp_task_wdt_init(30, false);
  bubbleUniverseHalf(0);
  xSemaphoreGive(semaphore);
  vTaskDelete(NULL);
}

void loop()
{
    int start = esp_timer_get_time();

    animTime = start * speed /1000;

    memset(offscreen, 0, pixelSize * pixelSize * sizeof(uint8_t));

    xTaskCreatePinnedToCore(halfTask, "Half", 1000, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
    bubbleUniverseHalf(1);
    xSemaphoreTake(semaphore, portMAX_DELAY);

    canvas.clear();
    canvas.drawBitmap(0,0, &bitmap);
    canvas.waitCompletion(true);
}
