import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  emoji: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    emoji: '🤖',
    description: (
      <>
        Master the Robot Operating System for real-time robot control and communication.
        Learn nodes, topics, services, and actions.
      </>
    ),
    link: '/docs/module-1-ros2',
  },
  {
    title: 'Simulation & Digital Twins',
    emoji: '🌐',
    description: (
      <>
        Build virtual robots in Gazebo and Unity for safe testing and development.
        Create physics-accurate digital twins.
      </>
    ),
    link: '/docs/module-2-simulation',
  },
  {
    title: 'NVIDIA Isaac Platform',
    emoji: '⚡',
    description: (
      <>
        Leverage GPU-accelerated robotics with Isaac Sim and ROS integration.
        Train AI models in photorealistic environments.
      </>
    ),
    link: '/docs/module-3-isaac',
  },
  {
    title: 'Vision-Language-Action',
    emoji: '🧠',
    description: (
      <>
        Implement cutting-edge VLA models for intelligent robot behavior.
        Bridge perception, language understanding, and physical actions.
      </>
    ),
    link: '/docs/module-4-vla',
  },
];

function Feature({title, emoji, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
      <Link to={link} className={styles.featureCard}>
        <div className={styles.featureEmoji}>{emoji}</div>
        <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
        <p className={styles.featureDescription}>{description}</p>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          What You'll Learn
        </Heading>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
