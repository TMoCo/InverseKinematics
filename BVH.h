/**
***  BVH����t�@�C���̓ǂݍ��݁E�`��N���X
***  Copyright (c) 2004-2007, Masaki OSHITA (www.oshita-lab.org)
***  Provided by Dr. He Wang for Animation and Simulation 2nd Assignment
***  Comments written by Thomas Moreno Cooper
**/

#pragma warning(disable: 4018)
#ifndef  _BVH_H_
#define  _BVH_H_


#include <vector>
#include <map>
#include <string>


using namespace  std;



//
//  BVH Class
//
class  BVH
{
  public:

	// Each channel is a degree of freedom. The first three are positional and the second three are angular
	enum  ChannelEnum
	{
		X_ROTATION, Y_ROTATION, Z_ROTATION,
		X_POSITION, Y_POSITION, Z_POSITION
	};
	struct  Joint;

	// struct for a single channel
	struct  Channel
	{
		// Pointer to a joint object (which joint does this channel belong to)
		Joint *              joint;
		
		// Enum for channel type
		ChannelEnum          type;

		// the channel's index
		int                  index;
	};

	// struct for a single joint
	struct  Joint
	{
		// the joint's name
		string               name;
		// the joint's index
		int                  index;

		// pointer to the joint's parent (another joint)
		Joint *              parent;
		// vetor of joint pointers, the joint's children, this is how we build a tree data structure 
		vector< Joint * >    children;

		// the joint's offset, how much we translate by to get the joint's position from parent position
		double               offset[3];

		// basically tells us when to end recursion when reading/rendering BVH
		bool                 has_site;
		// dunno
		double               site[3];

		// vector of pointers to the channels the joint has (up to 6)
		vector< Channel * >  channels;
	};


  public:
	// flag set for (un)succesful BVH file loading
	bool                     is_load_success;

	/* file data */
	string                   file_name;   // the name of the file
	string                   motion_name; // the motion's name (parsed file name)

	/*  BVH file header data  */
	int                      num_channel; // number of channels in a figure
	vector< Channel * >      channels;    // vector containing all channels
	vector< Joint * >        joints;      // vector containing all joints
	map< string, Joint * >   joint_index; // maps a joint's name to a joint

	/*  BVH file motion data  */
	int                      num_frame;   // the number of frames
	double                   interval;    // the time interval between each frame (desired FPS)
	double *                 motion;      // an array of doubles containing the values for each channel at each frame of the motion


  public:
	// constructors
	BVH();
	BVH( const char * bvh_file_name );
	~BVH();

	// destructor
	void  Clear();

	// loads a BVH file
	void  Load( const char * bvh_file_name );

  public:
	/*  methods */

	// returns the state of file load
	bool  IsLoadSuccess() const { return is_load_success; }

	// returns file data
	const string &  GetFileName() const { return file_name; }
	const string &  GetMotionName() const { return motion_name; }

	// returns BVH file header data
	const int       GetNumJoint() const { return  joints.size(); }
	const Joint *   GetJoint( int no ) const { return  joints[no]; }
	const int       GetNumChannel() const { return  channels.size(); }
	const Channel * GetChannel( int no ) const { return  channels[no]; }

	const Joint *   GetJoint( const string & j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }
	const Joint *   GetJoint( const char * j ) const  {
		map< string, Joint * >::const_iterator  i = joint_index.find( j );
		return  ( i != joint_index.end() ) ? (*i).second : NULL; }

	// returns BVH file motion data
	int     GetNumFrame() const { return  num_frame; }
	double  GetInterval() const { return  interval; }
	double  GetMotion( int f, int c ) const { return  motion[ f*num_channel + c ]; }

	// method for setting bvh motion data
	void  SetMotion( int f, int c, double v ) { motion[ f*num_channel + c ] = v; }

  public:
	/*  �p���̕`��֐�  */
	
	// �w��t���[���̎p����`��
	void  RenderFigure( int frame_no, float scale = 1.0f );

	// �w�肳�ꂽBVH���i�E�p����`��i�N���X�֐��j
	static void  RenderFigure( const Joint * root, const double * data, float scale = 1.0f );

	// BVH���i�̂P�{�̃����N��`��i�N���X�֐��j
	static void  RenderBone( float x0, float y0, float z0, float x1, float y1, float z1, float bRadius = 0.1 );
};



#endif // _BVH_H_
