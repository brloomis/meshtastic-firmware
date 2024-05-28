#include "configuration.h"

#pragma once
#include "FSCommon.h"
#include "GPSStatus.h"
#include "OLEDDisplay.h"
#include "OLEDDisplayUi.h"
#include "ProtobufModule.h"
#include "concurrency/OSThread.h"
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>

/**
 * Module to record data to an SD card
 */
class SDRecordModule : private concurrency::OSThread, public MeshModule
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

    void toggleRec();
    void shutdown();

    int32_t tryWritePos(int32_t myLat, int32_t myLon, int32_t myAlt, uint32_t numSats);

#if HAS_SCREEN

    virtual bool wantUIFrame() { return true; }

    virtual void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) override;
#endif

    virtual bool wantPacket(const meshtastic_MeshPacket *p) { return false; }

  protected:
    /** Does our periodic broadcast */
    virtual int32_t runOnce() override;

  private:
    bool openTrackFileByTime(time_t curtime);
    bool openTrackFileByPrefix(const String prefix);
    String genFileName(const String prefix, uint8_t num, const String suffix);
    int32_t writePos(int32_t myLat, int32_t myLon, int32_t myAlt, uint32_t numSats, time_t timestamp);

    bool writeFooter();

    File m_fp;
    bool m_fp_open;
    bool m_noCard;
    bool m_cantWrite;

    int m_lastLat;
    int m_lastLon;
    int m_lastAlt;
    time_t m_lastTime;
    uint32_t m_pointsRecorded;
    String m_fileName;
    String m_dbgdata;
    bool m_wantRecord;
};

extern SDRecordModule *sdRecordModule;