import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

const ModuleCard = ({ title, description, link, icon }) => (
  <div className="col col--3">
    <div className="card">
      <div className="card__header">
        <h3>{title}</h3>
      </div>
      <div className="card__body">
        <p>{description}</p>
      </div>
      <div className="card__footer">
        <Link to={link} className="button button--primary">
          Explore
        </Link>
      </div>
    </div>
  </div>
);

function ModuleCards() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          <ModuleCard
            title="Module 1: ROS 2 Nervous System"
            description="Learn about ROS 2 as the robot nervous system, nodes, topics, services, and actions."
            link="/docs/module1-ros2-nervous-system"
          />
          <ModuleCard
            title="Module 2: Digital Twin Simulation"
            description="Explore Gazebo physics simulation, Unity visualization, and digital twin concepts."
            link="/docs/module2-digital-twin-simulation"
          />
          <ModuleCard
            title="Module 3: AI Brain (NVIDIA Isaac)"
            description="Discover NVIDIA Isaac Sim, Isaac ROS perception stack, and AI-powered robotics."
            link="/docs/module3-ai-brain-isaac"
          />
          <ModuleCard
            title="Module 4: Vision-Language-Action Robotics"
            description="Understand VLA systems, integrating LLMs with robotics, and language-driven control."
            link="/docs/module4-vla-robotics"
          />
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <HomepageHeader />
      <main>
        <ModuleCards />
      </main>
    </Layout>
  );
}