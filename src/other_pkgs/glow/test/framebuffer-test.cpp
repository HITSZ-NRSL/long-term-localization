#include <glow/GlFramebuffer.h>
#include <glow/GlRenderbuffer.h>
#include <glow/GlState.h>
#include <glow/GlTexture.h>
#include <glow/GlTextureRectangle.h>
#include <gtest/gtest.h>

using namespace glow;

namespace {

TEST(FrambufferTest, initTest) {
  GlFramebuffer buf(10, 10);
  GlRenderbuffer rbo(10, 10, RenderbufferFormat::DEPTH_STENCIL);
  buf.attach(FramebufferAttachment::DEPTH_STENCIL, rbo);
  // note: as always...not a framebuffer until first time bound.
  buf.bind();
  buf.release();

  ASSERT_TRUE(glIsFramebuffer(buf.id()));
  ASSERT_NO_THROW(CheckGlError());
}

TEST(FramebufferTest, testStatelessness) {
  GlState priorState = GlState::queryAll();
  CheckGlError();

  GlFramebuffer buf(640, 480);
  if (priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
  ASSERT_EQ(true, (priorState == GlState::queryAll()));

  GlTexture texture(640, 480, TextureFormat::RGBA);
  GlRenderbuffer rbo(640, 480, RenderbufferFormat::DEPTH_STENCIL);
  if (priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
  ASSERT_EQ(true, (priorState == GlState::queryAll()));

  buf.attach(FramebufferAttachment::COLOR0, texture);
  //  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  //  glFinish();
  if (priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
  ASSERT_EQ(true, (priorState == GlState::queryAll()));

  buf.attach(FramebufferAttachment::DEPTH_STENCIL, rbo);
  //  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  //  glFinish();
  if (priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
  ASSERT_EQ(true, (priorState == GlState::queryAll()));

  buf.bind();

  buf.release();
  if (priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
  ASSERT_EQ(true, (priorState == GlState::queryAll()));

  buf.valid();
  if (priorState != GlState::queryAll()) priorState.difference(GlState::queryAll());
  ASSERT_EQ(true, (priorState == GlState::queryAll()));
}
}
