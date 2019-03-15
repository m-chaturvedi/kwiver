/*ckwg +29
 * Copyright 2018-2019 by Kitware, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation file for video input using GDAL.
 */

#include "mie4nitf_init.h"
#include "mie4nitf_video_input.h"

#include <vital/types/timestamp.h>
#include <vital/exceptions/io.h>
#include <vital/exceptions/video.h>
#include <vital/util/tokenize.h>
#include <vital/types/image_container.h>
#include <arrows/gdal/image_io.h>
#include <arrows/gdal/image_container.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/xmlstring.h>

#include <kwiversys/SystemTools.hxx>

#include <mutex>
#include <memory>
#include <vector>
#include <sstream>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

namespace kwiver {
namespace arrows {
namespace mie4nitf {

struct xml_metadata_per_frame {
  const char* start_timestamp;
  const char* end_timestamp;
  const char* filename;
  const char* description;
};

// ------------------------------------------------------------------
// Private implementation class
class mie4nitf_video_input::priv
{
public:
  /// Constructor
  priv() :
    frame(nullptr),
    start_time(-1),
    video_path(""),
    end_of_video(true),
    number_of_frames(0),
    gdal_mie4nitf_dataset_(nullptr)
  {}

  vital::image_container_sptr frame;

  // Start time of the stream, to offset the pts when computing the frame number.
  // (in stream time base)
  int64_t start_time;

  // Name of video we opened
  std::string video_path;

  // For logging in priv methods
  vital::logger_handle_t logger;
  static std::mutex open_mutex;

  bool end_of_video;
  size_t number_of_frames;
  std::shared_ptr<GDALDataset> gdal_mie4nitf_dataset_;
  std::vector<xml_metadata_per_frame> xml_metadata;

  // ==================================================================
  /*
  * @brief Whether the video was opened.
  *
  * @return \b true if video was opened.
  */
  bool is_opened()
  {
    return this->gdal_mie4nitf_dataset_ != nullptr;
  }

  xmlDoc *xml_read_input(const char *input, bool is_file=true) {
    xmlDoc *doc = NULL;
    xmlNode *root_element = NULL;

    if (is_file) {
      if ((doc = xmlReadFile(input, NULL, 0)) == NULL) {
        printf("error: could not parse file %s\n", input);
        exit(-1);
      }
    }
    /*
    else {
      if ((doc = xmlReadDoc(BAD_CAST input, NULL, NULL, 0)) == NULL) {
        printf("error: could not read input string");
        exit(-1);
    }
    */
    return doc;
  }


//  xmlChar *get_attribute_value(const char *attr, xmlXPathContextPtr context) {
//  std::string str = "./field[@name='" + std::string(attr) + "']";
//  xmlChar* xml_str = reinterpret_cast<const char*> (str.c_str());
//  xmlXPathObjectPtr xpath = get_node_set_from_context(
//      reinterpret_cast<char *>(xml_str), context);
//  assert(xpath -> nodesetval -> nodeNr == 1);
//  xmlChar* prop = xmlGetProp(xpath->nodesetval->nodeTab[0], BAD_CAST "value");
//  if(prop == NULL) {
//    xmlFree(prop);
//    printf("error: In xmlGetProp\n");
//    exit(-1);
//  }
//  xmlXPathFreeObject(xpath);
//  return prop;
//}
//
//MetadataPerFrame get_attributes_per_frame(xmlXPathContextPtr context) {
//  MetadataPerFrame md;
//  xmlChar* st = get_attribute_value("START_TIMESTAMP", context);
//  xmlChar* en = get_attribute_value("END_TIMESTAMP", context);
//  xmlChar* img = get_attribute_value("IMAGE_SEG_INDEX", context);
//
//  md.START_TIMESTAMP = (const char*)st;
//  md.END_TIMESTAMP = (const char*)en;
//  const char* ind = (const char*)img;
//  md.IMAGE_SEG_INDEX = std::stoi(ind);
//  xmlFree(reinterpret_cast<xmlChar*> ind);
//  return md;
//}
//
//void populate_frame_times(xmlDoc * const doc) {
//  xmlChar* TEMPORAL_BLOCK_XPATH =
//      reinterpret_cast<xmlChar*>("//tre[@name='MTIMFA']/repeated[@name='CAMERAS' and @number='1']/"
//      "group[@index='0']/repeated[@name='TEMPORAL_BLOCKS']/group");
//
//  xmlXPathContextPtr xpath_context = get_new_context(doc);
//  xmlXPathObjectPtr xpath_obj =
//      get_node_set_from_context(TEMPORAL_BLOCK_XPATH, xpath_context);
//
//  assert(xpath_obj -> nodesetval -> nodeNr == NUM_FRAMES_EXPECTED);
//
//  for (int i = 0; i < xpath_obj -> nodesetval -> nodeNr; ++i) {
//    xmlNode *node = xpath_obj->nodesetval->nodeTab[i];
//    xmlXPathContextPtr xpath_context_local = get_new_context(doc);
//
//    xmlXPathSetContextNode(node, xpath_context_local);
//
//    metadata_all_frames[i] = get_attributes_per_frame(xpath_context_local);
//    xmlXPathFreeContext(xpath_context_local);
//  }
//  xmlXPathFreeObject(xpath_obj);
//  xmlXPathFreeContext(xpath_context);
//};

  void populate_xml_metadata() {
    const char* tmp_data_file = "/tmp/mytre.xml";
    char **str = GDALGetMetadata(this->gdal_mie4nitf_dataset_.get(), "xml:TRE");
    FILE * pFile;
    pFile = fopen(tmp_data_file,"w");
    while(*str != NULL){
      fprintf(pFile, "%s\n", *str);
      str++;
    }
    fclose (pFile);

    xmlDoc *doc = xml_read_input(tmp_data_file, true);
    //populate_frame_times(doc);
  }
  // ==================================================================
  /*
  * @brief Open the given video MIE4NITF video.
  *
  * @return \b true if video was opened.
  */
   bool open(std::string video_name) {

    GDALAllRegister();

    gdal_mie4nitf_dataset_.reset(
      static_cast<GDALDataset*>(GDALOpen( video_name.c_str(), GA_ReadOnly ) ) );

    if ( !gdal_mie4nitf_dataset_ )
    {
      VITAL_THROW( vital::invalid_file, video_name, "GDAL could not load file.");
      return false;
    }
    populate_xml_metadata();
    return true;
  }

}; // end of internal class.

// static open interlocking mutex
std::mutex mie4nitf_video_input::priv::open_mutex;


// ==================================================================
mie4nitf_video_input
::mie4nitf_video_input()
  : d( new priv() )
{
  attach_logger( "mie4nitf_video_input" ); // get appropriate logger
  d->logger = this->logger();
  // Have been randomly set for now.
  this->set_capability(vital::algo::video_input::HAS_EOV, true);
  this->set_capability(vital::algo::video_input::HAS_FRAME_NUMBERS, true);
  this->set_capability(vital::algo::video_input::HAS_FRAME_DATA, true);
  this->set_capability(vital::algo::video_input::HAS_METADATA, false);

  this->set_capability(vital::algo::video_input::HAS_FRAME_TIME, false);
  this->set_capability(vital::algo::video_input::HAS_ABSOLUTE_FRAME_TIME, false);
  this->set_capability(vital::algo::video_input::HAS_TIMEOUT, false);
  this->set_capability(vital::algo::video_input::IS_SEEKABLE, true);

  mie4nitf_init();
}


mie4nitf_video_input
::~mie4nitf_video_input()
{
  return;
}


// ------------------------------------------------------------------
// Get this algorithm's \link vital::config_block configuration block \endlink
vital::config_block_sptr
mie4nitf_video_input
::get_configuration() const
{
  // get base config from base class
  vital::config_block_sptr config = vital::algo::video_input::get_configuration();

  return config;
}

// ------------------------------------------------------------------
// Set this algorithm's properties via a config block
void
mie4nitf_video_input
::set_configuration(vital::config_block_sptr in_config)
{
  // Starting with our generated vital::config_block to ensure that assumed values are present
  // An alternative is to check for key presence before performing a get_value() call.

  vital::config_block_sptr config = this->get_configuration();
  config->merge_config(in_config);
  return;
}

// ------------------------------------------------------------------
bool
mie4nitf_video_input
::check_configuration(vital::config_block_sptr config) const
{
  bool retcode(true); // assume success

  return retcode;
}

// ------------------------------------------------------------------
void
mie4nitf_video_input
::open( std::string video_name )
{
  this->close();

  d->video_path = video_name;

  {
    std::lock_guard< std::mutex > lock(d->open_mutex);

    if (!kwiversys::SystemTools::FileExists(d->video_path))
    {
      // Throw exception
      throw kwiver::vital::file_not_found_exception(video_name, "File not found");
    }

    if (!d->open(video_name))
    {
      throw kwiver::vital::video_runtime_exception("Video stream open failed for unknown reasons");
    }
    d->end_of_video = false;
	return;
  }
}


// ------------------------------------------------------------------
void
mie4nitf_video_input
::close()
{
    d->start_time = -1;
    d->video_path = "";
    d->end_of_video = true;
    d->number_of_frames = 0;
    d->gdal_mie4nitf_dataset_.reset();
    d->frame.reset();
}

// ------------------------------------------------------------------
bool
mie4nitf_video_input
::next_frame( kwiver::vital::timestamp& ts,
              uint32_t timeout )
{
  return false;
}

// ------------------------------------------------------------------
bool mie4nitf_video_input::seek_frame(kwiver::vital::timestamp& ts,
  kwiver::vital::timestamp::frame_t frame_number,
  uint32_t timeout)
{
  return true;
}


// ------------------------------------------------------------------
kwiver::vital::image_container_sptr
mie4nitf_video_input
::frame_image( )
{
  std::shared_ptr<kwiver::vital::image_container> ptr;
  return ptr;
}


// ------------------------------------------------------------------
kwiver::vital::timestamp
mie4nitf_video_input
::frame_timestamp() const
{
  kwiver::vital::timestamp ts;
  return ts;
}


// ------------------------------------------------------------------
kwiver::vital::metadata_vector
mie4nitf_video_input
::frame_metadata()
{
  return kwiver::vital::metadata_vector();
}


// ------------------------------------------------------------------
kwiver::vital::metadata_map_sptr
mie4nitf_video_input
::metadata_map()
{
  std::shared_ptr<kwiver::vital::simple_metadata_map> ptr;
  return ptr;
}


// ------------------------------------------------------------------
bool
mie4nitf_video_input
::end_of_video() const
{
  return false;
}


// ------------------------------------------------------------------
bool
mie4nitf_video_input
::good() const
{
  return true;
}


// ------------------------------------------------------------------
bool
mie4nitf_video_input
::seekable() const
{
  return true;
}

// ------------------------------------------------------------------
size_t
mie4nitf_video_input
::private_num_frames()
{
  return 0;
}

} } } // end namespaces
