#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#if 0
#include <loop_closure/ImageUtils.h>
#include <loop_closure/ImageFeatureGenerator.h>

class FeatureMatcher
{

public:

  /* *********************************************************************************** */
  typedef boost::shared_ptr<FeatureMatcher> Ptr;
  typedef boost::shared_ptr<const FeatureMatcher> ConstPtr;

  /* *********************************************************************************** */
  FeatureMatcher()
  {}

  /* *********************************************************************************** */
  ~FeatureMatcher()
  {}

  /* *********************************************************************************** */
  bool operator()(const ImageUtils::Features& f1,
                  const ImageUtils::Features& f2,
                  ImageUtils::Matches& out,
                  const std::string& matcher = std::string("FlannBased"))
  {
    // Create the descriptor matcher
    cv::Ptr<cv::DescriptorMatcher> matcher_ptr =
      cv::DescriptorMatcher::create(matcher);

    ImageUtils::Matches rough_matches;

    // Match descriptors between images
    matcher_ptr->match(f1.descriptors, f2.descriptors, rough_matches);

    // Refine with a min dist filter
    float min_dist = std::numeric_limits<float>::infinity();
    for (unsigned int ii = 0; ii < f1.descriptors.rows; ++ii)
    {
      float dist = rough_matches[ii].distance;
      min_dist = dist < min_dist ? dist : min_dist;
    }

    for (unsigned int ii = 0; ii < f1.descriptors.rows; ++ii)
      if (rough_matches[ii].distance <= std::max(2.f * min_dist, 0.02f))
        out.push_back(rough_matches[ii]);

    // Return whether there are matches or not
    return !out.empty();
  }

  /* *********************************************************************************** */
  bool operator()(const cv::Mat& im1,
                  const cv::Mat& im2,
                  ImageUtils::Matches& out,
                  const std::string& detector = std::string("SIFT"),
                  const std::string& descriptor = std::string("SIFT"),
                  const std::string& matcher = std::string("FlannBased"))
  {
    // Get features from an image first, then match them with operator()
    ImageFeatureGenerator ifg(detector, descriptor);
    ImageUtils::Features f1 = ifg(im1);
    ImageUtils::Features f2 = ifg(im2);

    return (*this)(f1, f2, out);
  }

  /* *********************************************************************************** */
  bool operator()(const sensor_msgs::Image& im1,
                  const sensor_msgs::Image& im2,
                  ImageUtils::Matches& out,
                  const std::string& detector = std::string("SIFT"),
                  const std::string& descriptor = std::string("SIFT"),
                  const std::string& matcher = std::string("FlannBased"))
  {
    // Get features from an image first, then match them with operator()
    ImageFeatureGenerator ifg(detector, descriptor);
    ImageUtils::Features f1 = ifg(im1);
    ImageUtils::Features f2 = ifg(im2);

    return (*this)(f1, f2, out);
  }

  /* *********************************************************************************** */
  void draw(const ImageUtils::Features& f1,
            const ImageUtils::Features& f2,
            const cv::Mat& image1,
            const cv::Mat& image2,
            const ImageUtils::Matches& matches,
            const std::string& window = std::string("Feature pairs"),
            const bool wait = true)
  {
    // Draw the feature matches on the two images
    cv::Mat display;
    cv::drawMatches(image1, f1.keypoints,
                    image2, f2.keypoints,
                    matches,
                    display,
                    cv::Scalar::all(-1),
                    cv::Scalar::all(-1),
                    std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow(window, display);

    if (wait)
      cv::waitKey(0); // wait for user keypress
    else
      cv::waitKey(30); // 30 ms pause
  }

private:

};
#endif
#endif
