#include "SDRecordModule.h"
#include "GPS.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "TypeConversions.h"
#include "airtime.h"
#include "configuration.h"
#include "gps/GeoCoord.h"
#include "sleep.h"
#include "target_specific.h"

#include "../graphics/ScreenFonts.h"
#include "FSCommon.h"
#include "PowerStatus.h"
#include "RTC.h"
#include "SD.h"

#define FILE_SUFFIX ".gpx"

// const char *TRACK_DIR = "/tracks/";
const char *GPX_HEADER = "<gpx version=\"1.1\" creator=\"SDRecordModule\">\n<trk><trkseg>\n";
const char *GPX_FOOTER = "</trkseg></trk></gpx>\n";

#define MIN_RECORD_INTERVAL (60)
#define RUNONCE_LOCK_INTERVAL 1000;
#define RUNONCE_NO_LOCK_INTERVAL 10000;

SDRecordModule *sdRecordModule;

SDRecordModule::SDRecordModule()
    : concurrency::OSThread("SDRecordModule"), m_fp(), m_fp_open(false), m_noCard(false), m_cantWrite(false), m_lastLat(0),
      m_lastLon(0), m_lastAlt(0), m_lastTime(0), m_pointsRecorded(0), m_dbgdata(), m_wantRecord(true)

{
    LOG_DEBUG("SDRecord is UP\n");
    setup();
}
SDRecordModule::~SDRecordModule()
{
    LOG_DEBUG("Shutting down...\n");
    shutdown();
}

void SDRecordModule::setup()
{
    if (nullptr != gps) {
        LOG_DEBUG("SDRecord::setup going to observe gps status\n");
        gpsStatusObserver.observe(&gps->newStatus);
    } else {
        LOG_DEBUG("SDRecord::setup gps is nullptr!\n");
    }
}

String SDRecordModule::genFileName(const String prefix, uint8_t num, const String suffix)
{
    char buf[4] = {0};
    snprintf(buf, sizeof(buf), "%03u", num);
    String output = prefix + buf + suffix;

    return output;
}

bool SDRecordModule::openTrackFileByTime(time_t curtime)
{
    struct tm *t = NULL;
    char timebuf[20] = {0};

    if (0L >= curtime) {
        LOG_ERROR("SDRecord- ts_unix invalid: 0x%08X\n", curtime);
        m_dbgdata = String("curtime zero");
        return false;
    }
    t = gmtime(&curtime);
    if (!t) {
        LOG_ERROR("SDRecord- gmtime failed\n");
        return false;
    }

    // TODO hardcoded constant
    if (120 >= t->tm_year) {
        LOG_ERROR("SDRecord- gmtime year is bad: %d\n", t->tm_year);
        return false;
    }

    size_t sfret = strftime(timebuf, sizeof(timebuf), "%Y%m%d_", t);
    if (0 == sfret || sizeof(timebuf) <= sfret) {
        LOG_ERROR("SDRecord- strftime failed: %lu\n", sfret);
        return false;
    }

    LOG_DEBUG("SDRecord- Got timestamp prefix: %s\n", timebuf);
    return openTrackFileByPrefix(timebuf);
}

bool SDRecordModule::openTrackFileByPrefix(const String prefix)
{
    LOG_ERROR("SDRecord- cardSize = %llu\n", SD.cardSize());
    LOG_ERROR("SDRecord- cardType = %d\n", SD.cardType());

    if (CARD_NONE == SD.cardType() || 0LLU == SD.cardSize()) {
        LOG_ERROR("SDRecord- No card or type is none!\n");
        m_noCard = true;
        return false;
    }

    if (m_fp_open) {
        LOG_DEBUG("SDRecord- Tried to openTrackFile but m_fp_open was set!\n");
        m_fp.close();
        m_fp_open = false;
    }

    // check if the /tracks dir exists
    LOG_DEBUG("SDRecord- Looking for track files...\n");
    bool trackdir_exists = SD.exists("/tracks");
    if (!trackdir_exists) {
        LOG_DEBUG("SDRecord- /tracks didn't exist, making dir!\n");
        bool mkdir_ret = SD.mkdir("/tracks");
        if (!mkdir_ret) {
            LOG_ERROR("SDRecord- Could not mkdir /tracks\n");
        }
    }

    File trackdir = SD.open("/tracks", FILE_O_READ);
    if (!trackdir || !trackdir.isDirectory()) {
        LOG_ERROR("SDRecord- Could not open /tracks a second time or it wasn't a dir!\n");
        m_cantWrite = true;
        return false;
    }
    if (trackdir) {
        trackdir.close();
    }

    // open the log file
    uint8_t log_num = 0;
    String track_filename = "/tracks/" + genFileName(prefix, log_num, FILE_SUFFIX);
    LOG_DEBUG("SDRecord- Checking file: %s\n", track_filename.c_str());
    bool file_exists = SD.exists(track_filename);
    while (file_exists && log_num < 254) {
        LOG_DEBUG("SDRecord- File exists!\n");
        log_num = log_num + 1;
        track_filename = "/tracks/" + genFileName(prefix, log_num, FILE_SUFFIX);
        LOG_DEBUG("SDRecord- Now checking file: %s\n", track_filename.c_str());
        file_exists = SD.exists(track_filename);
    }
    if (file_exists) {
        LOG_ERROR("SDRecord- Could not find a free filename!\n");
    } else {
        LOG_DEBUG("SDRecord- Filename %s is free\n", track_filename.c_str());
        LOG_DEBUG("**SDRecord- m_fp was: %p %d\n", m_fp, m_fp.available());
        if (m_fp.name()) {
            LOG_DEBUG("*** we're trying to open m_fp but it already had a name: %s\n", m_fp.name());
        } else {
            LOG_DEBUG("*** m_fp.name is false\n");
        }
        m_fp = SD.open(track_filename, FILE_O_WRITE, true);
        LOG_DEBUG("**SDRecord- m_fp now: %p %d %s\n", m_fp, m_fp.available(), m_fp.name());
        if (nullptr == m_fp.name()) {
            LOG_ERROR("SDRecord- Could not open %s for writing\n", track_filename.c_str());
            return false;
        }
        m_fp_open = true;
        m_fileName = track_filename;
        // write the header
        m_fp.write(reinterpret_cast<const uint8_t *>(GPX_HEADER), strlen(GPX_HEADER));
    }
    return true;
}

int32_t SDRecordModule::runOnce()
{
    return 0;
    // meshtastic_NodeInfoLite *node = nodeDB.getMeshNode(nodeDB.getNodeNum());

    // LOG_DEBUG("SDRecord- lat: %u, lon: %u, time: %u\n", node->position.latitude_i, node->position.longitude_i,
    // node->position.time); writePos();
    if (gpsStatus->getHasLock()) {
        LOG_DEBUG("SDRecord- runOnce - has lock, writing pos\n");
        // writePos();
        return RUNONCE_LOCK_INTERVAL;
    } else {
        LOG_DEBUG("SDRecord - runOnce - no lock!\n");
        return RUNONCE_NO_LOCK_INTERVAL;
    }
}

int32_t SDRecordModule::tryWritePos(int32_t myLat, int32_t myLon, int32_t myAlt, uint32_t numSats)
{

    // LOG_DEBUG("SDRecord- ****** %s\n", __FUNCTION__);

    if (m_cantWrite || m_noCard) {
        // LOG_DEBUG("SDRecord- Dropping writePos, m_cantWrite = %d, m_noCard = %d\n", m_cantWrite, m_noCard);
        return 0;
    }
    uint32_t rtc_time = getValidTime(RTCQualityGPS);

    LOG_DEBUG("SDRecord tryWritePos got lat: %d, lon: %d, alt: %d, numsats: %u\n", myLat, myLon, myAlt, numSats);

    LOG_DEBUG("SDRecord time: %lu\n", rtc_time);
    if (0 == myLat || (0 == myLon) || (0 == numSats)) {
        LOG_ERROR("SDRecord- tryWritePos was called but lat=%d, lon=%d, numSats=%lu\n", myLat, myLon, numSats);
        return -1;
    }
    if (m_fp_open) {
        if ((m_lastTime != rtc_time) && ((myLat != m_lastLat) || (myLon != m_lastLon) || (myAlt != m_lastAlt) ||
                                         ((rtc_time - MIN_RECORD_INTERVAL) > m_lastTime))) {
            if ((rtc_time - MIN_RECORD_INTERVAL) > m_lastTime) {
                LOG_DEBUG("SDRecord- More than %u seconds passed since last write\n", MIN_RECORD_INTERVAL);
            }
            if ((myLat == m_lastLat) && (myLon == m_lastLon) && (myAlt == m_lastAlt)) {
                LOG_DEBUG("SDRecord- Have not moved since last write\n");
            } else {
                LOG_DEBUG("SDRecord- Diff: lat=%d, lon=%d, alt=%d\n", abs(myLat - m_lastLat), abs(myLon - m_lastLon),
                          abs(myAlt - m_lastAlt));
            }

            return this->writePos(myLat, myLon, myAlt, numSats, static_cast<time_t>(rtc_time));

        } else {
            if (m_lastTime == rtc_time) {
                LOG_DEBUG("SDRecord- Less than a second since last write\n");
            }
            if ((myLat == m_lastLat) && (myLon == m_lastLon) && (myAlt == m_lastAlt)) {
                LOG_DEBUG("SDRecord- Did not move since last write\n");
            }
        }

    } else {
        LOG_DEBUG("SDRecord- tryWritePos: fp was closed\n");
    }

    return 0;
}

int32_t SDRecordModule::writePos(int32_t myLat, int32_t myLon, int32_t myAlt, uint32_t numSats, time_t timestamp)
{
    if (m_fp_open) {
        struct tm *t = gmtime(&timestamp);
        if (!t) {
            LOG_ERROR("SDRecord- gmtime returned %p\n", t);
            return -1;
        }
        // TODO const
        if (120 >= t->tm_year) {
            LOG_ERROR("SDRecord- gmtime year is bad: %d\n", t->tm_year);
            return -1;
        }
        char buf[100] = {0};

        // TODO check returns
        snprintf(buf, sizeof(buf), "<trkpt lat=\"%.7f\" lon=\"%.7f\">\n", myLat * 1e-7, myLon * 1e-7);

        m_fp.write(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        memset(buf, 0x0, sizeof(buf));

        snprintf(buf, sizeof(buf), "    <ele>%d</ele>\n", myAlt);
        m_fp.write(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        memset(buf, 0x0, sizeof(buf));

        strftime(buf, sizeof(buf), "    <time>%Y-%m-%dT%H:%M:%SZ</time>\n", t);
        m_fp.write(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        memset(buf, 0x0, sizeof(buf));

        snprintf(buf, sizeof(buf), "    <extensions>\n");
        m_fp.write(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        memset(buf, 0x0, sizeof(buf));

        snprintf(buf, sizeof(buf), "        <brl batMv=\"%d\" batPct=\"%hu\" numSats=\"%u\"> </brl>\n",
                 powerStatus->getBatteryVoltageMv(), powerStatus->getBatteryChargePercent(), numSats);
        m_fp.write(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        memset(buf, 0x0, sizeof(buf));

        snprintf(buf, sizeof(buf), "    </extensions>\n</trkpt>\n");
        m_fp.write(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        memset(buf, 0x0, sizeof(buf));

        m_fp.flush();
        LOG_DEBUG("SDRecord- Data was written!\n");
        m_pointsRecorded++;
        m_lastLat = myLat;
        m_lastLon = myLon;
        m_lastAlt = myAlt;
        m_lastTime = timestamp;
    } else {
        LOG_ERROR("SDRecord- writePos was called but m_fp_open was false\n");
        return -1;
    }

    return 0;
}
void SDRecordModule::toggleRec()
{
    LOG_ERROR("SDRecord- m_wantRecord was: %d\n", m_wantRecord);
    if (m_wantRecord) {
        m_wantRecord = false;
        shutdown();
    } else {
        m_wantRecord = true;
    }
    LOG_ERROR("SDRecord- m_wantRecord now: %d\n", m_wantRecord);
}

void SDRecordModule::shutdown()
{
    if (m_fp_open) {
        LOG_DEBUG("SDRecord- Closing m_fp\n");
        m_fp.write(reinterpret_cast<const uint8_t *>(GPX_FOOTER), strlen(GPX_FOOTER));
        m_fp.flush();
        m_fp.close();
        m_fp_open = false;
        m_cantWrite = false;
        m_noCard = false;
        m_pointsRecorded = 0;
        m_fileName = "";
    } else {
        LOG_DEBUG("SDRecord- wanted to shut down but not running\n");
    }
}

int SDRecordModule::handleStatusUpdate(const meshtastic::GPSStatus *newStatus)
{
    // LOG_DEBUG("SDRecord- ****** %s\n", __FUNCTION__);

    if (m_cantWrite || m_noCard) {
        // LOG_DEBUG("SDRecord- Dropping handleStatusUpdate, m_cantWrite = %d, m_noCard = %d\n", m_cantWrite, m_noCard);

        return 0;
    }
    if (!m_wantRecord) {
        LOG_DEBUG("SDRecord- disabled\n");
        return 0;
    }

    uint32_t rtc_time = getValidTime(RTCQualityGPS);

    LOG_DEBUG("SDRecord got status update %p\n", newStatus);
    if (newStatus->getHasLock()) {
        // load data from GPS object, will add timestamp + battery further down
        LOG_DEBUG("SDRecord- We have lock! rtc_time is 0x%08X\n", rtc_time);
        if (0 != rtc_time) {
            if (!m_fp_open) {
                bool ret = this->openTrackFileByTime(static_cast<time_t>(rtc_time));
                if (false == ret) {
                    LOG_ERROR("SDRecord- openTrackFileByTime returned false\n");
                }

            } else {
                // dbgtime[0] = '2';
                // LOG_DEBUG("SDRecord- fp was already open\n");
            }
            if (m_fp_open) {
                int32_t myLat = newStatus->getLatitude();
                int32_t myLon = newStatus->getLongitude();
                int32_t myAlt = newStatus->getAltitude();
                uint32_t numSats = newStatus->getNumSatellites();

                /*
                    LOG_DEBUG("SDRecord *** update, fp is open,  got lat: %d, lon: %d, alt: %d, numsats: %u\n", myLat, myLon,
                    myAlt, numSats);
                */
                // write pos here
                int32_t writeret = this->tryWritePos(myLat, myLon, myAlt, numSats);
                LOG_DEBUG("SDRecord- tryWritePos ret %d\n", writeret);
            } else {
                LOG_DEBUG("SDRecord- fp didn't open, not writing pos\n");
            }

        } else {
            LOG_DEBUG("SDRecord- No lock :(\n");
        }
    } else {
        LOG_DEBUG("SDRecord newStatus has no lock: %d\n", newStatus->getHasLock());
    }
    return 0;
}

void SDRecordModule::drawFrameRecorder(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    String fileprefix("F: ");
    char points[20] = {0};
    uint16_t header_offset = 0;

    display->setFont(FONT_SMALL);
    display->setColor(WHITE);

    // The coordinates define the left starting point of the text
    display->setTextAlignment(TEXT_ALIGN_LEFT);

    if (config.display.displaymode == meshtastic_Config_DisplayConfig_DisplayMode_INVERTED) {
        display->fillRect(0 + x, 0 + y, x + display->getWidth(), y + FONT_HEIGHT_SMALL);
        display->setColor(BLACK);
    }
    if (m_fp_open) {
        display->drawString(x, y, String("Rec "));
        header_offset = display->getStringWidth("Rec ");

        display->drawString(x, y + FONT_HEIGHT_SMALL, fileprefix + m_fileName);
        snprintf(points, sizeof(points), "%lu pts", m_pointsRecorded);
        display->drawString(x + header_offset, y, points);

    } else {
        if (!m_wantRecord) {
            display->drawString(x, y, String("SD Off "));
            header_offset = display->getStringWidth("SD Off ");

        } else {
            display->drawString(x, y, String("SD Idle "));
            header_offset = display->getStringWidth("SD Idle ");
        }
    }
    if (m_noCard) {
        display->drawString(x + header_offset, y, String("No Card"));
    } else if (m_cantWrite) {
        display->drawString(x + header_offset, y, String("Error"));
    }
    if (0 != m_dbgdata.length()) {
        display->drawString(x, y + (FONT_HEIGHT_SMALL * 3), m_dbgdata);
    } else {
        display->drawString(x, y + (FONT_HEIGHT_SMALL * 3), String("no dbg"));
    }
}