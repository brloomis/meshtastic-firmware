#pragma once
#include "FSCommon.h"
#include "GPSStatus.h"
#include "OLEDDisplay.h"
#include "OLEDDisplayUi.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"

/**
 * Module to record data to an SD card
 */
class SDRecordModule : private concurrency::OSThread
{
    CallbackObserver<SDRecordModule, const meshtastic::GPSStatus *> gpsStatusObserver =
        CallbackObserver<SDRecordModule, const meshtastic::GPSStatus *>(this, &SDRecordModule::handleStatusUpdate);

    uint32_t lastStore = 0;

  public:
    /** Constructor
     * name is for debugging output
     */
    SDRecordModule();
    ~SDRecordModule();
    void setup();

    int handleStatusUpdate(const meshtastic::GPSStatus *newStatus);

    void shutdown();

    int32_t writePos(int32_t myLat, int32_t myLon, int32_t myAlt, uint32_t numSats);

    void drawFrameRecorder(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y);

  protected:
    /** Does our periodic broadcast */
    virtual int32_t runOnce() override;

  private:
    bool openTrackFile(const String prefix);
    String genFileName(const String prefix, uint8_t num, const String suffix);

    File m_fp;
    bool m_fp_open;
    bool m_noCard;
    bool m_cantWrite;

    int m_lastLat;
    int m_lastLon;
    int m_lastAlt;
    uint32_t m_lastTime;
    uint32_t m_pointsRecorded;
    String m_fileName;
};

extern SDRecordModule *sdRecordModule;