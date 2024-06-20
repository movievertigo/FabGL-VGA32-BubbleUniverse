#include "fabgl.h"
#include <esp_task_wdt.h>

#define SS_640x480_135Hz  "\"640x480@135Hz\" 60 640 680 744 848  480 483 487 523 -HSync -VSync"
#define SS_800x600_90Hz  "\"800x600@90Hz\" 60 800 840 968 1056 600 601 605 628 -HSync -VSync"
#define SS_1024x768_60Hz "\"1024x768@60Hz\" 62.5 1024 1072 1176 1308 768 771 775 795 -HSync -VSync"

const char* screenMode = SS_1024x768_60Hz;
const int width = 1024;
const int pixelSize = 768;
const int xOffset = (width - pixelSize)/2;
const float pSize = 0.25f * pixelSize;
const float pOffset = 0.5f * pixelSize;
const int sinTableSize = 1<<13;
const float sinTableScale = sinTableSize / (2*PI) / pSize;

float* sinTable;
uint8_t* offscreen;

fabgl::VGADirectController displayController;
TaskHandle_t  mainTaskHandle;
SemaphoreHandle_t semaphore;

const int curveShift = 8;
const int curveCount = 1<<curveShift;
const int curveStep = 4;
const int iterationShift = 9;
const int iterationCount = 1<<iterationShift;

const int slotSizeMultiplier = 1.55f * iterationCount * curveCount / (curveStep * pixelSize);
const int minSlotSize = 2;

struct Slot
{
    uint8_t capacity;
    uint8_t count;
    uint16_t* data;
};

Slot* writeSlots = nullptr;
Slot* displaySlots = nullptr;
Slot* slotsA = nullptr;
Slot* slotsB = nullptr;

float speed = 0.00015f;

const float ang1inc = pSize * curveStep * 2 * PI / 235;
const float ang2inc = pSize * curveStep;
float animTime;

bool vsync = false;
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

void createSlots()
{
    slotsA = (Slot*)malloc(pixelSize * sizeof(Slot));
    slotsB = (Slot*)malloc(pixelSize * sizeof(Slot));
    for (int i = 0; i < pixelSize; ++i)
    {
        for (int bank = 0; bank < 2; ++bank)
        {
            Slot* slots = bank ? slotsB : slotsA;

            int r = pixelSize>>1;
            int slotSize = slotSizeMultiplier * sqrt(r*r - (i+0.5f-r)*(i+0.5f-r)) / r;
            slotSize = slotSize < minSlotSize ? minSlotSize : slotSize;
            slots[i].capacity = slotSize;
            slots[i].count = 0;
            slots[i].data = (uint16_t*)malloc(slotSize * sizeof(uint16_t));
            if (slots[i].data == nullptr)
            {
                Serial.println("Failed to allocate memory for slot");
            }
        }
    }

    writeSlots = slotsA;
    displaySlots = slotsB;
}

void clearSlots()
{
    for (int i = 0; i < pixelSize; ++i)
    {
        writeSlots[i].count = 0;
    }
}

void IRAM_ATTR drawScanline(void * arg, uint8_t * dest, int scanLine)
{
    if (displaySlots)
    {
        memset(dest, displayController.createBlankRawPixel(), width);
        const Slot& slot = displaySlots[scanLine];
        for (int i = 0; i < slot.count; ++i)
        {
            auto pixel = displayController.createBlankRawPixel() | (slot.data[i] >> 10);
            VGA_PIXELINROW(dest, xOffset + (slot.data[i] & ((1<<10)-1))) = pixel;
        }
        
        if (scanLine == pixelSize-1)
        {
            vTaskNotifyGiveFromISR(mainTaskHandle, NULL); // Trigger vsync
        }
    }
}

void setup(void)
{
    Serial.begin(115200);

    mainTaskHandle = xTaskGetCurrentTaskHandle();

    displayController.begin();
    displayController.setScanlinesPerCallBack(1);
    displayController.setDrawScanlineCallback(drawScanline);
    displayController.setResolution(screenMode, width, pixelSize, false);

    createSinTable();
    createSlots();

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
            Slot& slot = writeSlots[(int)y];
            if (slot.count < slot.capacity)
            {
                bool found = false;
                for (int check = 0; check < slot.count; ++check)
                {
                    if ((slot.data[check] & ((1<<10)-1)) == (int)x)
                    {
                        slot.data[check] = (int)x + (red << 10) + (green << 12) + (blue << 14);
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    slot.data[slot.count++] = (int)x + (red << 10) + (green << 12) + (blue << 14);
                }
            }
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

    clearSlots();

    xTaskCreatePinnedToCore(halfTask, "Half", 1000, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
    bubbleUniverseHalf(1);
    xSemaphoreTake(semaphore, portMAX_DELAY);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for vsync

    writeSlots = (writeSlots == slotsA ? slotsB : slotsA);
    displaySlots = (displaySlots == slotsA ? slotsB : slotsA);
}
