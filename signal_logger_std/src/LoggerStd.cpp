/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014,  Christian Gehring, Michael Bloesch,
 * Peter Fankhauser, C. Dario Bellicoso
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
* @file 	LoggerSL.cpp
* @author 	Christian Gehring, C. Dario Bellicoso
* @date		July 7, 2013
* @version 	1.0
* @ingroup 	signal_logger_std
* @brief  Copied and adapted from SL.
*/
#include "signal_logger_std/LoggerStd.hpp"
#include <roco/log/log_messages.hpp>
#include <cassert>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* byte swapping routines */
#define BYTESWAP

#ifdef  alpha
#define BYTESWAP
#endif

#ifdef i386
#define BYTESWAP
#endif

#ifdef i386mac
#define BYTESWAP
#endif

#ifdef i486
#define BYTESWAP
#endif

#ifdef i486xeno
#define BYTESWAP
#endif

#ifdef i586
#define BYTESWAP
#endif

#ifdef x86_64
#define BYTESWAP
#endif

#ifdef x86_64mac
#define BYTESWAP
#endif

#ifdef x86_64xeno
#define BYTESWAP
#endif



#ifndef WORDSWAP

#define MSB(x)	(((x) >> 8) & 0xff)  /*!< most signif byte of 2-byte integer */
#define LSB(x)	((x) & 0xff)	     /*!< least signif byte of 2-byte integer*/
#define MSW(x) (((x) >> 16) & 0xffff)/*!< most signif word of 2-word integer */
#define LSW(x) ((x) & 0xffff) 	     /*!< least signif byte of 2-word integer*/

#define WORDSWAP(x) (MSW(x) | (LSW(x) << 16))

#define LLSB(x)	((x) & 0xff)		/*!< 32bit word byte/word swap macros */
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x)	 (((x) >> 24) & 0xff)

#define LONGSWAP(x) ((LLSB(x) << 24) |		\
		     (LNLSB(x) << 16)|		\
		     (LNMSB(x) << 8) |		\
		     (LMSB(x)))

#endif


namespace signal_logger_std {

LoggerStd::LoggerStd(const std::string& filename):
    signal_logger::LoggerBase(),
    filename_(filename),
		nMaxVariables_(1000),
		store_in_buffer_rate(0),
		n_save_data_points(1),
		n_data_in_buffer(0),
		save_data_flag(false),
		n_cvars(0),
		n_cols(0),
		n_rows(0),
		n_calls(0),
		file_number(0),
		data_collect_time(0.0),
		buffer(NULL),
		isUpdateLocked_(false),
		isUpdatingLoggingScript_(false),
		filenamePrefix("d"),
		isReadingFileNumberFromFile_(true),
		outputLevel_(VerboseLevel::VL_INFO)
{

}

LoggerStd::~LoggerStd() {
  // free memory
  if (buffer != NULL) {
	//my_free_fmatrix(buffer,1,n_rows,1,n_cols);
	deallocateBuffer(buffer, n_rows, n_cols);
	buffer = NULL;
  }
}

const signal_logger::LoggerBase::LoggerType LoggerStd::getLoggerType() const {
  return signal_logger::LoggerBase::LoggerType::TypeStd;
}

void LoggerStd::setFilenamePrefix(const std::string& prefix) {
  filenamePrefix = prefix;
}

void LoggerStd::setFileName(const std::string& filename) {
  filename_ = filename;
}

void LoggerStd::setIsReadingFileNumberFromFile(bool isReading) {
  isReadingFileNumberFromFile_ = isReading;
}

void LoggerStd::setInitialFileNumber(int number) {
  file_number = number;
}

void LoggerStd::setVerboseLevel(VerboseLevel level) {
  outputLevel_ = level;
}


void LoggerStd::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) {
	ROCO_INFO("LoggerStd::initLogger");
  setFileName(logScriptFileName);
  init(updateFrequency, samplingFrequency, samplingTime);
}

void LoggerStd::init(int updateFrequency, int samplingFrequency, double samplingTime)
{
	assert(samplingFrequency<=updateFrequency);
	assert(samplingFrequency>0);
	assert(updateFrequency>0);

	updateFrequency_ = updateFrequency;
	samplingFrequency_ = samplingFrequency;
	samplingTime_ = samplingTime;

	store_in_buffer_rate = (double) updateFrequency_ / (double) samplingFrequency_;
	n_save_data_points	= samplingTime_ /((double) store_in_buffer_rate) * updateFrequency_;
//	printf("n_save_data_points: %d store_in_buffer_rate: %d\n", n_save_data_points, store_in_buffer_rate);

	/* the first variable is always the time */
	vars.ptr   = (char *) &data_collect_time;
	strcpy(vars.name,"time");
	strcpy(vars.units,"s");
	vars.type  = VT_DOUBLE;
//	vars.nptr   = NULL;
//	vars.nptr.reset(NULL);

	isInitialized_ = true;
	isInitialized_ = readDataCollectScript(filename_, false);
	if (!isInitialized_) {
	  ROCO_WARN("LoggerStd: Could not initialize!");
	}

}

void LoggerStd::lockUpdate()
{
	isUpdateLocked_ = true;
}
bool LoggerStd::readDataCollectScript(std::string fname, bool flag )
{

  FILE  *infile;
  int    i,rc;
  char   string[100];
  Cinfo *ptr;

 if (!isInitialized_) {
    printf("Collect data is not initialized\n");
    return false;
  }



  infile = fopen(fname.c_str(),"r");

  if (infile == NULL) {
    printf("Error when trying to open file >%s< !\n",fname.c_str());
    return false;
  }

  /* make sure no data collection is running */
  save_data_flag = false;

  /* now read the file */
  n_cvars = 0;
  while ((rc=fscanf(infile,"%s",string)) != EOF) {

    /* figure out whether the string exists */
    ptr = &vars;
    do {
      if (strcmp(ptr->name,string) == 0) {
	break;
      }
//      ptr = (Cinfo *)ptr->nptr;
      ptr = ptr->nptr.get();
    } while (ptr);

    if (!ptr)
      continue;

    /* otherwise, the appropriate variable was found and needs to be
       added to the data collection array */

    if (n_cvars >= nMaxVariables_) {
      printf("Memory for number of variables in collect_data exhausted\n");
      break;
    }

    cvars[++n_cvars] = ptr;
//    printf("add variable %s\n", ptr->name);

  }

  fclose(infile);

  if (flag) {
    printf("\nRead %d valid names from file >%s< .\n\n",n_cvars,fname.c_str());
  }

  /* ensure that we have an adequate data collection buffer */
  if (buffer != NULL) {
	  // free memory
    //my_free_fmatrix(buffer,1,n_rows,1,n_cols);
	deallocateBuffer(buffer, n_rows, n_cols);
    buffer = NULL;
  }

  if (n_cvars > 0 && n_save_data_points > 0) {
    n_rows = n_save_data_points;
    n_cols = n_cvars;

    /* create buffer by allocating memory */
    //  buffer = my_fmatrix(1,n_rows,1,n_cols);
    buffer= allocateBuffer(n_rows, n_cols);


  }

  return true;

}

float** LoggerStd::allocateBuffer(int nrh, int nch)
{
	int nrl = 1;
	int ncl = 1;
    int info = 0;
    if (nrl==1 && ncl == 1) {
      info = 1;
    }

    void  *rc;
    rc = calloc((size_t)(nrh-nrl+1+info),sizeof(float*));
    assert(rc != NULL);
    float **m;
    m = (float **) rc;


    if (info) {

        m[0] = (float *) calloc((size_t) 3,sizeof(float));
       assert(m[0]!=NULL);
        m[0][2] = 1;
        m[0][01]       = nrh-nrl+1;
        m[0][1]       = nch-ncl+1;

      }


    float  *chunk;
    chunk = (float *) calloc((size_t) (nrh-nrl+1) * (nch-ncl+1),sizeof(float));
    assert(chunk != NULL);

    for(int i=nrl;i<=nrh;i++) {
      m[i]=(float *) &(chunk[(i-nrl)*(nch-ncl+1)]);
      m[i] -= ncl;
    }
    return m;
}

void LoggerStd::deallocateBuffer(float** mat, int nrh, int nch)
{
	 int nrl = 1;
	 int ncl = 1;

	free((char*) &(mat[nrl][ncl]));
	if (nrl==1 && ncl==1) {
	free((char*) mat[0]);
	free((char*) mat);
	} else {
	free((char*) (mat+nrl));
	}
}

void LoggerStd::startLogger()
{
	if (!isInitialized_) {
	  if (outputLevel_ != VerboseLevel::VL_NONE) {
	    printf("Logger is not initialized!\n");
	  }
		return;
	}

	save_data_flag      = true;
	n_calls             = 0;
	data_collect_time   = 0;
	n_data_in_buffer    = 0;
}
void LoggerStd::stopLogger()
{
	if (!isInitialized_) {
		return;
	}

  save_data_flag = false;
  if (outputLevel_ != VerboseLevel::VL_NONE) {
    printf("time=%d.%03d : Data collection interrupted!\n",
           (int) data_collect_time,
           ((int) (data_collect_time * 1000)) - ((int) data_collect_time) * 1000);
  }
}

void LoggerStd::collectLoggerData()
{

	  int i,j,n;
	  float temp;

	if (!isInitialized_) {
		return;
	}

	  if (save_data_flag && n_calls%store_in_buffer_rate == 0 &&
	      n_save_data_points > n_data_in_buffer)  {

	    ++n_data_in_buffer;

	    for (i=1; i<=n_cvars; ++i) {

	      switch (cvars[i]->type) {

	      case VT_DOUBLE:
		buffer[n_data_in_buffer][i] =
		  (float) *((double *) cvars[i]->ptr);
		break;

	      case VT_FLOAT:
		buffer[n_data_in_buffer][i] =
		  (float) *((float *) cvars[i]->ptr);
		break;

	      case VT_INT:
		buffer[n_data_in_buffer][i] =
		  (float) *((int *) cvars[i]->ptr);
		break;

	      case VT_SHORT:
		buffer[n_data_in_buffer][i] =
		  (float) *((short *) cvars[i]->ptr);
		break;

	      case VT_LONG:
		buffer[n_data_in_buffer][i] =
		  (float) *((long *) cvars[i]->ptr);
		break;

	      case VT_CHAR:
		buffer[n_data_in_buffer][i] =
		  (float) *((char *) cvars[i]->ptr);
		break;
	      case VT_BOOL:
     buffer[n_data_in_buffer][i] =
      (float) *((bool *) cvars[i]->ptr);
    break;
	      }

	    }

	    if (n_data_in_buffer >= n_save_data_points) {
	      save_data_flag = false;
	      printf("time=%d.%03d : buffer is full!\n", (int) data_collect_time,
		     ((int)(data_collect_time*1000))-((int) data_collect_time)*1000);
	    }

	  }

	  /* increment the counter of calls to this function */
	  ++n_calls;
	  data_collect_time = ((double)n_calls) / updateFrequency_;

}

void LoggerStd::saveLoggerData()
{
	int  i,j,rc;
	FILE *fp, *fd, *in;
	char filename[100];
	int  buffer_size;
	int  aux;


	if (!isInitialized_) {
	  if (outputLevel_ != VerboseLevel::VL_NONE) {
	    printf("Collect data is not initialized\n");
	  }
		return;
	}

	if (save_data_flag) {
	  if (outputLevel_ != VerboseLevel::VL_NONE) {
	    printf("Data collection is running -- try saving later\n");
	  }
		return;
	}

	if (n_cvars < 1) {
	  if (outputLevel_ != VerboseLevel::VL_NONE) {
	    printf("No variables specified for Data collection -- try outMenu\n");
	  }
		return;
	}
	if (outputLevel_ != VerboseLevel::VL_NONE) {
	  printf( "Saving data:\n" );
	}
	// create a file name
//	  if ( ( in = fopen( "last_data", "r" ) ) != NULL ) { // migration to .last_data
//	    int rc;
//	    rc = system("mv last_data .last_data");
//	    fclose(in);
//	  }

    if (isReadingFileNumberFromFile_) {
      if ( ( in = fopen( ".last_data", "r" ) ) == NULL )   {
          printf( "cannot fopen file .last_data for read.\n" );
      } else {
        rc=fscanf( in, "%d\n", &file_number );
        fclose( in );
      }
    }

	  sprintf( filename, "%s%05d", filenamePrefix.c_str(), file_number );
	  printf( "Saving data in %s\n", filename );
//	  if (isReadingFileNumberFromFile) {
	    file_number++;
//	  }

	  if ( ( fp = fopen( filename, "w" ) ) == NULL )  {
	    printf( "cannot fopen file %s for write.\n", filename );
	    return;
	  }

	  /* the buffer size, the number of columns, the sampling time,
	     the column names and units */

	  buffer_size = n_save_data_points * n_cvars;
	  fprintf(fp,"%d  %d %d  %f\n",buffer_size, n_cvars,
		  n_save_data_points,
		  updateFrequency_/(double)store_in_buffer_rate);

	  for (i=1; i<=n_cvars; ++i) {
	    fprintf(fp,"%s  ",cvars[i]->name);
	    fprintf(fp,"%s  ",cvars[i]->units);
	  }
	  fprintf(fp,"\n");

	#ifdef BYTESWAP
	  /* convert little-endian to big-endian */
	  for (j=1; j<=n_cols; ++j) {
	    for (i=1; i<=n_rows; ++i) {
	      aux = LONGSWAP(*((int *)&(buffer[i][j])));
	      buffer[i][j] = *((float *)&aux);
	    }
	  }
	#endif

	  if (fwrite(&(buffer[1][1]),sizeof(float),n_rows*n_cols,fp)!= n_rows*n_cols) {
	    printf( "cannot fwrite matrix.\n" );
	    return;
	  }

	  fclose( fp );

	  if (isReadingFileNumberFromFile_) {
      if ( ( fd = fopen( ".last_data", "w" ) ) == NULL )   {
          printf( "cannot fopen file .last_data for write.\n" );
          return;
      }
      fprintf( fd, "%d\n", file_number );
      fclose( fd );
	  }

	  /* clear buffer for next accumulation */
	   for (int i=1; i<=buffer[0][0]; ++i) {
	      for (int j=1; j<=buffer[0][1]; ++j) {
	    	  buffer[i][j] = 0.0f;
	      }
	    }

	   if (outputLevel_ != VerboseLevel::VL_NONE) {
	     printf( "All done, captain!\n" );
	   }
}

void LoggerStd::addFloatToLog(const float& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
	if (!isInitialized_) {
		return;
	}
	std::string varName = group + name;
	//addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_FLOAT, (int) update);
	addVarToCollect((char*)(&const_cast<float&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_FLOAT, (int) update);
}
void LoggerStd::addDoubleToLog(const double& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
	if (!isInitialized_) {
		return;
	}
	std::string varName = group + name;
	//addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_DOUBLE, (int) update);
	addVarToCollect((char*)(&const_cast<double&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_DOUBLE, (int) update);
}
void LoggerStd::addIntToLog(const int& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
	if (!isInitialized_) {
		return;
	}
	std::string varName = group + name;
	//addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_INT, (int) update);
	addVarToCollect((char*)(&const_cast<int&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_INT, (int) update);
}
void LoggerStd::addShortToLog(const short& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
	if (!isInitialized_) {
		return;
	}
	std::string varName = group + name;
	//addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_SHORT, (int) update);
	addVarToCollect((char*)(&const_cast<short&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_SHORT, (int) update);
}
void LoggerStd::addLongToLog(const long& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
	if (!isInitialized_) {
		return;
	}
	std::string varName = group + name;
	//addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_LONG, (int) update);
	addVarToCollect((char*)(&const_cast<long&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_LONG, (int) update);
}
void LoggerStd::addCharToLog(const char& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
	if (!isInitialized_) {
		return;
	}
	std::string varName = group + name;
	//addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_CHAR, (int) update);
	addVarToCollect((char*)(&const_cast<char&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_CHAR, (int) update);

}


void LoggerStd::addBoolToLog(const bool& var, const std::string& name, const std::string& group, const std::string& unit, bool update)
{
  if (!isInitialized_) {
    return;
  }
  std::string varName = group + name;
  //addVarToCollect((char*) &var, (char*)varName.c_str(), (char*)unit.c_str(), VT_BOOL, (int) update);
  addVarToCollect((char*)(&const_cast<bool&>(var)), (char*)varName.c_str(), (char*)unit.c_str(), VT_BOOL, (int) update);

}



/*****************************
 * Add Eigen matrices to log *
 *****************************/
void LoggerStd::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addDoubleToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addFloatToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addIntToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addShortToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addLongToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addCharToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addCharToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addBoolToLog(var(r,c), name_of_var, group, unit, update);
    }
  }
}

void LoggerStd::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(var(0), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(var(1), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(var(2), std::string{name + "_z"}, group, unit, update);
}


void LoggerStd::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addDoubleToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addFloatToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addIntToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addShortToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addLongToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addCharToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addCharToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addBoolToLog(var(r,c), names(r,c), group, unit, update);
    }
  }
}

void LoggerStd::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(var(0), names(0,0), group, unit, update);
  addDoubleToLog(var(1), names(1,0), group, unit, update);
  addDoubleToLog(var(2), names(2,0), group, unit, update);
}
/*****************************/


/******************
 * Kindr wrappers *
 ******************/
void LoggerStd::addDoubleKindrPositionToLog(const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(position.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(position.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(position.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(rotation.toImplementation().w(), std::string{name + "_w"}, group, unit, update);
  addDoubleToLog(rotation.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(rotation.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(rotation.toImplementation().x(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(rotation.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(rotation.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(rotation.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(angVel.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(angVel.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(angVel.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(angleAxis.toImplementation().axis().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(angleAxis.toImplementation().axis().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(angleAxis.toImplementation().axis().z(), std::string{name + "_z"}, group, unit, update);
  addDoubleToLog(angleAxis.toImplementation().angle(), std::string{name + "_angle"}, group, unit, update);
}

void LoggerStd::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleEigenMatrixToLog(const_cast<KindrRotationMatrixD&>(rotMat).toImplementation(), name, group, unit, update);
}

void LoggerStd::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(rotVec.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(rotVec.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(rotVec.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(linVel.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(linVel.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(linVel.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(linAcc.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(linAcc.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(linAcc.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(angAcc.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(angAcc.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(angAcc.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrForceToLog(const KindrForceD& force, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(force.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(force.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(force.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrTorqueToLog(const KindrTorqueD& torque, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(torque.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(torque.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(torque.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrVectorToLog(const KindrVectorD& vector, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(vector.toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(vector.toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(vector.toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerStd::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& vectorFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) { }
void LoggerStd::addDoubleKindrForceAtPositionToLog( const KindrForceD& force,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& forceFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) { }
void LoggerStd::addDoubleKindrTorqueAtPositionToLog(const KindrTorqueD& torque,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& torqueFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) { }

/******************/


//void LoggerStd::addCharEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<char>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}
//
//void LoggerStd::addFloatEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<float>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}
//void LoggerStd::addIntEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<int>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}
//void LoggerStd::addShortEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<short>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}
//void LoggerStd::addLongEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<long>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}
//void LoggerStd::addCharEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<char>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}
//void LoggerStd::addBoolEigenVectorToLog(const Eigen::Ref<Eigen::MatrixBase<bool>>& var, const std::string& name, const std::string& group = std::string {"/logging/" }, const std::string& unit = "-", bool update = false) {
//  Eigen::Matrix<std::string, 3, 1> names;
//  names(0) = name + "_x";
//  names(1) = name + "_y";
//  names(2) = name + "_z";
//  addToLog(var, names, group, unit, update);
//}



void LoggerStd::addVarToCollect(char *vptr,char *name,char *units, int type, int flag)
{

  Cinfo *ptr;
  FILE  *outfile;
  char   string[100];

  if (!isInitialized_) {
    if (outputLevel_ != VerboseLevel::VL_NONE) {
      printf("Collect data is not initialized\n");
    }
    return;
  }

  ptr = &vars;

  while (ptr != NULL) {
    /* does variable already exist? */
    if (strcmp(ptr->name,name)==0) {
      /* overwrite the pointer in case a task was re-initialized */
      ptr->ptr  = vptr;
      return;
    }
    if (!ptr->nptr)
      break;
//    ptr = (Cinfo *)ptr->nptr;
    ptr = ptr->nptr.get();
  }
  const int maxChars = LOGGER_STD_MAXCHARS;

//  ptr->nptr = (char *) my_calloc(1,sizeof(Cinfo),MY_STOP);
  ptr->nptr.reset(new Cinfo);
//  ptr = (Cinfo *)ptr->nptr;
  ptr = ptr->nptr.get();
  ptr->ptr  = vptr;
//  ptr->nptr = NULL;
  ptr->type = type;
  strncpy(ptr->name,name,maxChars-1);
  ptr->name[maxChars-1] = '\0';
  strncpy(ptr->units,units,maxChars-1);
  ptr->units[maxChars-1] = '\0';

  if (flag) {
	  readDataCollectScript(filename_,false);
  }
}

void LoggerStd::updateLogger(bool updateScript)
{
    isUpdatingLoggingScript_ = updateScript;

	  if (!isInitialized_) {
	    if (outputLevel_ != VerboseLevel::VL_NONE) {
	      printf("Collect data is not initialized\n");
	    }
	    return;
	  }

	  if (isUpdateLocked_) {
		  return;
	  }

	  if (isUpdatingLoggingScript_) {

      FILE *outfile;
      Cinfo *ptr;
      /* dump all variables to the sample.script */

      outfile = fopen(filename_.c_str(),"w");
      if (outfile == NULL) {
        printf("Couldn't open file >%s< for append\n",filename_.c_str());
        return;
      }

      ptr = &vars;
      while (ptr != NULL) {
        fprintf(outfile,"%s\n",ptr->name);
        ptr = (Cinfo *)ptr->nptr.get();
      }
      fclose(outfile);

	  }

	  readDataCollectScript(filename_,false);
}

void LoggerStd::stopAndSaveLoggerData() {
  LoggerStd::stopLogger();
  LoggerStd::saveLoggerData();
}

void LoggerStd::addTimestampToLog(const TimestampPair& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addLongToLog(var.first, name+std::string{"_sec"}, group, std::string{"sec"}, update);
  addLongToLog(var.second, name+std::string{"_nsec"}, group, std::string{"nsec"}, update);
}


} /* namespace robotUtils */
