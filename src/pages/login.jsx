import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

const LoginPage = () => {
  return (
    <Layout title="Login" description="Login to your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Login to Your Account</h2>
              </div>
              <div className="card__body">
                <form>
                  <div className="form-group margin-bottom--md">
                    <label htmlFor="email">Email:</label>
                    <input
                      type="email"
                      id="email"
                      className="form-control"
                      placeholder="Enter your email"
                      required
                    />
                  </div>
                  <div className="form-group margin-bottom--md">
                    <label htmlFor="password">Password:</label>
                    <input
                      type="password"
                      id="password"
                      className="form-control"
                      placeholder="Enter your password"
                      required
                    />
                  </div>
                  <div className="margin-bottom--md">
                    <button type="submit" className="button button--primary button--block">
                      Login
                    </button>
                  </div>
                </form>

                <div className="text--center margin-bottom--md">
                  <p>
                    Don't have an account?{' '}
                    <Link to="/signup">
                      <strong>Sign up here</strong>
                    </Link>
                  </p>
                </div>

                <div className="text--center">
                  <Link to="/auth" className="button button--secondary button--sm">
                    Back to Main Auth
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default LoginPage;